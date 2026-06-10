// ── ROS connection ─────────────────────────────────────────────────────────────

const ros = new ROSLIB.Ros({ url: `ws://${window.location.hostname}:9090` });

const elStatus      = document.getElementById('connection-status');
const elNodeState   = document.getElementById('node-state');
const elDriveStatus = document.getElementById('drive-status');
const elServiceMsg  = document.getElementById('service-msg');

ros.on('connection', () => {
  elStatus.textContent = 'ROS Connected';
  elStatus.className = 'status connected';
  refreshNodeState();
});
ros.on('error',  () => { elStatus.textContent = 'ROS Error';        elStatus.className = 'status disconnected'; });
ros.on('close',  () => { elStatus.textContent = 'ROS Disconnected'; elStatus.className = 'status disconnected'; });

// ── Lifecycle ──────────────────────────────────────────────────────────────────

const TRANSITIONS = { configure: 1, activate: 3, deactivate: 4 };

function changeState(transitionId) {
  new ROSLIB.Service({
    ros,
    name: '/mserve_drivechain/change_state',
    serviceType: 'lifecycle_msgs/srv/ChangeState',
  }).callService(
    new ROSLIB.ServiceRequest({ transition: { id: transitionId, label: '' } }),
    () => setTimeout(refreshNodeState, 400),
    (err) => console.error('change_state error', err),
  );
}

function refreshNodeState() {
  new ROSLIB.Service({
    ros,
    name: '/mserve_drivechain/get_state',
    serviceType: 'lifecycle_msgs/srv/GetState',
  }).callService(new ROSLIB.ServiceRequest({}), (res) => {
    const label = res?.current_state?.label ?? 'unavailable';
    elNodeState.textContent = label;
    elNodeState.className = `state-label state-${label}`;
  }, () => {
    elNodeState.textContent = 'unavailable';
    elNodeState.className = 'state-label state-unavailable';
  });
}

document.getElementById('btn-configure').addEventListener('click',  () => changeState(TRANSITIONS.configure));
document.getElementById('btn-activate').addEventListener('click',   () => changeState(TRANSITIONS.activate));
document.getElementById('btn-deactivate').addEventListener('click', () => changeState(TRANSITIONS.deactivate));

setInterval(refreshNodeState, 4000);

// ── DriveChain services ────────────────────────────────────────────────────────

const connectSvc = new ROSLIB.Service({
  ros,
  name: '/mserve_drivechain/connect',
  serviceType: 'std_srvs/srv/Trigger',
});

const stopSvc = new ROSLIB.Service({
  ros,
  name: '/mserve_drivechain/stop',
  serviceType: 'std_srvs/srv/Trigger',
});

const driveSvc = new ROSLIB.Service({
  ros,
  name: '/mserve_drivechain/drive',
  serviceType: 'interfaces/srv/Drive',
});

const setMotorIdSvc = new ROSLIB.Service({
  ros,
  name: '/mserve_drivechain/set_motor_id',
  serviceType: 'interfaces/srv/SetMotorId',
});

function callConnect() {
  elServiceMsg.textContent = 'Calling…';
  connectSvc.callService(
    new ROSLIB.ServiceRequest({}),
    (res) => { elServiceMsg.textContent = res.message; },
    (err) => { elServiceMsg.textContent = `Error: ${err}`; },
  );
}

function callStop() {
  elServiceMsg.textContent = 'Calling…';
  stopSvc.callService(
    new ROSLIB.ServiceRequest({}),
    (res) => { elServiceMsg.textContent = res.message; },
    (err) => { elServiceMsg.textContent = `Error: ${err}`; },
  );
}

document.getElementById('btn-connect').addEventListener('click', callConnect);
document.getElementById('btn-hw-stop').addEventListener('click', callStop);

// ── Drive status subscription ──────────────────────────────────────────────────

new ROSLIB.Topic({
  ros,
  name: '/mserve_drivechain/drive_status',
  messageType: 'interfaces/msg/DriveStatus',
}).subscribe((msg) => {
  const s = msg.status ?? '—';
  elDriveStatus.textContent = s;
  elDriveStatus.className = `service-status ${s.replace('_', '-')}`;
});

// ── Motor config ───────────────────────────────────────────────────────────────

function getMotorConfig() {
  return [...document.querySelectorAll('#motor-table-body tr')]
    .map(row => ({
      id:      parseInt(row.querySelector('.motor-id').value),
      name:    row.querySelector('.motor-name').value.trim(),
      sign:    parseInt(row.querySelector('.motor-sign').value),
      enabled: row.querySelector('.motor-enabled').checked,
    }))
    .filter(m => m.enabled && m.id >= 1 && m.id <= 253);
}

// ── Motor commands ─────────────────────────────────────────────────────────────

// Read slider value for a motor by id; returns 0 if row not found.
function getSliderRpm(motorId) {
  const row = document.querySelector(`.motor-cmd-row[data-motor-id="${motorId}"]`);
  return row ? parseInt(row.querySelector('.motor-rpm-slider').value) : 0;
}

// Send all enabled motors at their current slider values via the ~/drive service.
function sendAll() {
  const motor_commands = getMotorConfig().map(m => ({
    motor_id: m.id,
    rpm: getSliderRpm(m.id),
  }));
  driveSvc.callService(
    new ROSLIB.ServiceRequest({ motor_commands }),
    () => {},
    (err) => console.error('drive error', err),
  );
}

// Zero one motor's slider then send all.
function stopMotor(motorId) {
  const row = document.querySelector(`.motor-cmd-row[data-motor-id="${motorId}"]`);
  if (row) {
    const slider = row.querySelector('.motor-rpm-slider');
    const display = row.querySelector('.motor-rpm-val');
    slider.value = '0';
    display.textContent = '0 rpm';
  }
  sendAll();
}

// ── Live mode ──────────────────────────────────────────────────────────────────

let liveInterval = null;

document.getElementById('chk-live').addEventListener('change', (e) => {
  if (e.target.checked) {
    liveInterval = setInterval(sendAll, 200);
  } else {
    if (liveInterval) { clearInterval(liveInterval); liveInterval = null; }
  }
});

// ── Stop All ───────────────────────────────────────────────────────────────────

document.getElementById('btn-stop-all').addEventListener('click', () => {
  document.querySelectorAll('.motor-rpm-slider').forEach(s => { s.value = '0'; });
  document.querySelectorAll('.motor-rpm-val').forEach(d => { d.textContent = '0 rpm'; });
  sendAll();
});

// ── Motor command row builder ──────────────────────────────────────────────────

function buildMotorControls() {
  const motors = getMotorConfig();
  const container = document.getElementById('motor-cmd-rows');
  container.innerHTML = '';

  if (motors.length === 0) {
    container.innerHTML = '<p class="motor-cmd-empty">No enabled motors — check motor config above.</p>';
    return;
  }

  motors.forEach(m => {
    const row = document.createElement('div');
    row.className = 'motor-cmd-row';
    row.dataset.motorId = m.id;

    const label = document.createElement('span');
    label.className = 'motor-cmd-label';
    label.innerHTML = `${escHtml(m.name)} <span class="motor-id-badge">#${m.id}</span>`;

    const slider = document.createElement('input');
    slider.type = 'range';
    slider.className = 'motor-rpm-slider';
    slider.min = '-200';
    slider.max = '200';
    slider.step = '1';
    slider.value = '0';

    const rpmDisplay = document.createElement('span');
    rpmDisplay.className = 'motor-rpm-val';
    rpmDisplay.textContent = '0 rpm';

    slider.addEventListener('input', () => {
      rpmDisplay.textContent = `${slider.value} rpm`;
    });

    const btnSend = document.createElement('button');
    btnSend.textContent = 'Send';
    btnSend.addEventListener('click', sendAll);

    const btnStop = document.createElement('button');
    btnStop.textContent = 'Stop';
    btnStop.className = 'btn-danger';
    btnStop.style.padding = '6px 12px';
    btnStop.addEventListener('click', () => stopMotor(m.id));

    row.append(label, slider, rpmDisplay, btnSend, btnStop);
    container.appendChild(row);
  });
}

document.getElementById('btn-apply-config').addEventListener('click', buildMotorControls);

// Build on first load
buildMotorControls();

// ── Motor feedback subscription ────────────────────────────────────────────────

const RAD_TO_DEG = 180 / Math.PI;

new ROSLIB.Topic({
  ros,
  name: '/mserve_drivechain/motor_feedback',
  messageType: 'interfaces/msg/DriveMotorFeedback',
}).subscribe((msg) => {
  const grid        = document.getElementById('feedback-grid');
  const placeholder = document.getElementById('feedback-placeholder');
  const motors      = msg.motors ?? [];

  placeholder.style.display = motors.length ? 'none' : 'block';

  const seen = new Set();
  motors.forEach(m => {
    const cardId = `fb-motor-${m.motor_id}`;
    seen.add(cardId);

    let card = document.getElementById(cardId);
    if (!card) {
      card = document.createElement('div');
      card.id = cardId;
      card.className = 'feedback-card';
      grid.appendChild(card);
    }

    const rads    = (m.velocity_rads ?? m.velocity_rpm * 2 * Math.PI / 60).toFixed(2);
    const deg     = (m.position_rad * RAD_TO_DEG).toFixed(1);
    const current = m.current_a     != null ? m.current_a.toFixed(2)     : '—';
    const temp    = m.temperature_c != null ? m.temperature_c.toFixed(1) : '—';
    const fault   = m.fault_code
      ? ` &nbsp;<span style="color:var(--danger)">FAULT ${m.fault_code}</span>` : '';
    const valStyle = m.fault_code ? ' style="color:var(--danger)"' : '';

    card.innerHTML =
      `<h4>${escHtml(m.name)} <span style="color:#475569">#${m.motor_id}</span></h4>` +
      `<div class="feedback-value"${valStyle}>${m.velocity_rpm.toFixed(0)}</div>` +
      `<div class="feedback-unit">rpm &nbsp;·&nbsp; ${rads} rad/s</div>` +
      `<div style="font-size:0.75rem;color:#64748b;margin-top:6px">` +
        `pos ${deg}° &nbsp; ${current} A &nbsp; ${temp} °C${fault}` +
      `</div>`;
  });

  [...grid.children].forEach(c => { if (!seen.has(c.id)) grid.removeChild(c); });
});

// ── /rosout log panel ──────────────────────────────────────────────────────────

const logBox      = document.getElementById('log-box');
const chkAllNodes = document.getElementById('chk-all-nodes');
const chkDebug    = document.getElementById('chk-debug');

document.getElementById('btn-log-clear').addEventListener('click', () => { logBox.innerHTML = ''; });

const LOG_LEVELS = {
  10: ['DEBUG','log-debug'], 20: ['INFO','log-info'],
  30: ['WARN','log-warn'],   40: ['ERROR','log-error'], 50: ['FATAL','log-fatal'],
};

new ROSLIB.Topic({
  ros,
  name: '/rosout',
  messageType: 'rcl_interfaces/msg/Log',
}).subscribe((msg) => {
  const level = msg.level ?? 20;
  if (level === 10 && !chkDebug.checked) return;
  const nodeName = msg.name ?? '';
  if (!chkAllNodes.checked && !nodeName.includes('mserve_drivechain')) return;

  const [levelLabel, levelClass] = LOG_LEVELS[level] ?? ['?', 'log-info'];
  const d  = new Date();
  const ts = `${String(d.getHours()).padStart(2,'0')}:${String(d.getMinutes()).padStart(2,'0')}:${String(d.getSeconds()).padStart(2,'0')}`;

  const line = document.createElement('div');
  line.className = levelClass;
  line.innerHTML =
    `<span class="log-ts">${ts}</span>` +
    `<span class="log-node">[${nodeName}]</span>` +
    `<span>[${levelLabel}] ${escHtml(msg.msg ?? '')}</span>`;
  logBox.appendChild(line);

  while (logBox.children.length > 500) logBox.removeChild(logBox.firstChild);
  if (logBox.scrollHeight - logBox.scrollTop < logBox.clientHeight + 60) {
    logBox.scrollTop = logBox.scrollHeight;
  }
});

function escHtml(s) {
  return String(s).replace(/&/g,'&amp;').replace(/</g,'&lt;').replace(/>/g,'&gt;');
}
