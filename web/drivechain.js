// ── ROS connection ────────────────────────────────────────────────────────────

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

// ── Lifecycle ─────────────────────────────────────────────────────────────────

const TRANSITIONS = { configure: 1, activate: 3, deactivate: 4 };

function changeState(transitionId) {
  const svc = new ROSLIB.Service({
    ros,
    name: '/mserve_drivechain/change_state',
    serviceType: 'lifecycle_msgs/srv/ChangeState',
  });
  svc.callService(
    new ROSLIB.ServiceRequest({ transition: { id: transitionId, label: '' } }),
    () => setTimeout(refreshNodeState, 400),
    (err) => console.error('change_state error', err),
  );
}

function refreshNodeState() {
  const svc = new ROSLIB.Service({
    ros,
    name: '/mserve_drivechain/get_state',
    serviceType: 'lifecycle_msgs/srv/GetState',
  });
  svc.callService(new ROSLIB.ServiceRequest({}), (res) => {
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

// ── DriveChain service ────────────────────────────────────────────────────────

const drivechainSvc = new ROSLIB.Service({
  ros,
  name: '/mserve_drivechain/drivechain_cmd',
  serviceType: 'interfaces/srv/DriveChainCmd',
});

function callDrivechain(command, motorId = 0, newId = 0) {
  elServiceMsg.textContent = 'Calling…';
  drivechainSvc.callService(
    new ROSLIB.ServiceRequest({ command, motor_id: motorId, new_id: newId }),
    (res) => { elServiceMsg.textContent = res.message; },
    (err) => { elServiceMsg.textContent = `Error: ${err}`; },
  );
}

document.getElementById('btn-connect').addEventListener('click',  () => callDrivechain(1));
document.getElementById('btn-hw-stop').addEventListener('click',  () => callDrivechain(2));

// ── Drive status subscription ─────────────────────────────────────────────────

const statusSub = new ROSLIB.Topic({
  ros,
  name: '/mserve_drivechain/drive_status',
  messageType: 'interfaces/msg/DriveStatus',
});

statusSub.subscribe((msg) => {
  const s = msg.status ?? '—';
  elDriveStatus.textContent = s;
  elDriveStatus.className = `service-status ${s.replace('_', '-')}`;
});

// ── Drive config helpers ──────────────────────────────────────────────────────

function getWheelConfig() {
  return {
    separation: parseFloat(document.getElementById('cfg-separation').value) || 0.35,
    radius:     parseFloat(document.getElementById('cfg-radius').value)     || 0.08,
    maxRpm:     parseInt(document.getElementById('cfg-max-rpm').value)      || 200,
  };
}

function getMotorConfig() {
  return [...document.querySelectorAll('#motor-table-body tr')].map(row => ({
    id:      parseInt(row.querySelector('.motor-id').value),
    name:    row.querySelector('.motor-name').value.trim(),
    side:    row.querySelector('.motor-side').value,
    enabled: row.querySelector('.motor-enabled').checked,
  })).filter(m => m.enabled && m.id >= 1 && m.id <= 253);
}

function diffRpm(linear, angular, separation, radius, maxRpm) {
  const half  = separation / 2;
  const toRpm = (vel) => Math.max(-maxRpm, Math.min(maxRpm,
    Math.round(vel / radius * 60 / (2 * Math.PI))));
  return { left: toRpm(linear - angular * half), right: toRpm(linear + angular * half) };
}

// ── Motor commands publisher ──────────────────────────────────────────────────

const motorCommandsTopic = new ROSLIB.Topic({
  ros,
  name: '/mserve_drivechain/motor_commands',
  messageType: 'interfaces/msg/MotorCommands',
});

function publishMotorCommands(linear, angular) {
  const { separation, radius, maxRpm } = getWheelConfig();
  const rpms     = diffRpm(linear, angular, separation, radius, maxRpm);
  const commands = getMotorConfig().map(m => ({
    motor_id: m.id,
    rpm: m.side === 'left' ? rpms.left : rpms.right,
  }));

  motorCommandsTopic.publish(new ROSLIB.Message({ commands }));

  const display = document.getElementById('rpm-display');
  display.style.display = commands.length ? 'block' : 'none';
  document.getElementById('rpm-left').textContent  = `L: ${rpms.left}`;
  document.getElementById('rpm-right').textContent = `R: ${rpms.right}`;
}

// ── Speed sliders ─────────────────────────────────────────────────────────────

const sliderLinear  = document.getElementById('slider-linear');
const sliderAngular = document.getElementById('slider-angular');
const valLinear     = document.getElementById('val-linear');
const valAngular    = document.getElementById('val-angular');

sliderLinear.addEventListener('input', () => {
  valLinear.textContent = `${Number(sliderLinear.value).toFixed(2)} m/s`;
});
sliderAngular.addEventListener('input', () => {
  valAngular.textContent = `${Number(sliderAngular.value).toFixed(2)} rad/s`;
});

function getLinear()  { return parseFloat(sliderLinear.value);  }
function getAngular() { return parseFloat(sliderAngular.value); }

// ── D-pad: hold-to-drive ──────────────────────────────────────────────────────

let driveInterval = null;
let activeDir = null;

const DIRS = {
  forward: () => publishMotorCommands( getLinear(), 0),
  back:    () => publishMotorCommands(-getLinear(), 0),
  left:    () => publishMotorCommands(0,  getAngular()),
  right:   () => publishMotorCommands(0, -getAngular()),
  stop:    () => publishMotorCommands(0, 0),
};

function startDriving(dir) {
  if (activeDir === dir) return;
  stopDriving();
  activeDir = dir;
  document.getElementById(`btn-${dir}`)?.classList.add('active');

  if (dir === 'stop') {
    publishMotorCommands(0, 0);
    return;
  }
  DIRS[dir]();
  driveInterval = setInterval(DIRS[dir], 100);
}

function stopDriving() {
  if (driveInterval) { clearInterval(driveInterval); driveInterval = null; }
  if (activeDir && activeDir !== 'stop') {
    document.getElementById(`btn-${activeDir}`)?.classList.remove('active');
    publishMotorCommands(0, 0);
  }
  activeDir = null;
}

// Mouse / touch
['forward', 'back', 'left', 'right', 'stop'].forEach((dir) => {
  const btn = document.getElementById(`btn-${dir}`);
  btn.addEventListener('mousedown',  () => startDriving(dir));
  btn.addEventListener('touchstart', (e) => { e.preventDefault(); startDriving(dir); }, { passive: false });
});

document.addEventListener('mouseup',  stopDriving);
document.addEventListener('touchend', stopDriving);

// Keyboard
const KEY_MAP = {
  ArrowUp:    'forward', KeyW: 'forward',
  ArrowDown:  'back',    KeyS: 'back',
  ArrowLeft:  'left',    KeyA: 'left',
  ArrowRight: 'right',   KeyD: 'right',
  Space:      'stop',
};

document.addEventListener('keydown', (e) => {
  const dir = KEY_MAP[e.code];
  if (!dir) return;
  e.preventDefault();
  startDriving(dir);
});

document.addEventListener('keyup', (e) => {
  const dir = KEY_MAP[e.code];
  if (!dir || dir === 'stop') return;
  stopDriving();
});

// ── Motor feedback subscription ───────────────────────────────────────────────

const feedbackSub = new ROSLIB.Topic({
  ros,
  name: '/mserve_drivechain/motor_feedback',
  messageType: 'interfaces/msg/DriveMotorFeedback',
});

const RAD_TO_DEG = 180 / Math.PI;

feedbackSub.subscribe((msg) => {
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

    const faultAttr = m.fault_code ? ' style="color:var(--danger)"' : '';
    const rads      = (m.velocity_rads ?? m.velocity_rpm * 2 * Math.PI / 60).toFixed(2);
    const deg       = (m.position_rad * RAD_TO_DEG).toFixed(1);
    const current   = m.current_a     != null ? m.current_a.toFixed(2)     : '—';
    const temp      = m.temperature_c != null ? m.temperature_c.toFixed(1) : '—';
    const fault     = m.fault_code
      ? ` &nbsp;<span style="color:var(--danger)">FAULT ${m.fault_code}</span>` : '';

    card.innerHTML =
      `<h4>${escHtml(m.name)} <span style="color:#475569">#${m.motor_id}</span></h4>` +
      `<div class="feedback-value"${faultAttr}>${m.velocity_rpm.toFixed(0)}</div>` +
      `<div class="feedback-unit">rpm &nbsp;·&nbsp; ${rads} rad/s</div>` +
      `<div style="font-size:0.75rem;color:#64748b;margin-top:6px">` +
        `pos ${deg}° &nbsp; ${current} A &nbsp; ${temp} °C${fault}` +
      `</div>`;
  });

  // Remove cards for motors no longer in the message
  [...grid.children].forEach(c => { if (!seen.has(c.id)) grid.removeChild(c); });
});

// ── Periodic state refresh ────────────────────────────────────────────────────

setInterval(refreshNodeState, 4000);

// ── /rosout log panel ─────────────────────────────────────────────────────────

const logBox      = document.getElementById('log-box');
const chkAllNodes = document.getElementById('chk-all-nodes');
const chkDebug    = document.getElementById('chk-debug');

document.getElementById('btn-log-clear').addEventListener('click', () => { logBox.innerHTML = ''; });

const LOG_LEVELS = { 10: ['DEBUG','log-debug'], 20: ['INFO','log-info'], 30: ['WARN','log-warn'], 40: ['ERROR','log-error'], 50: ['FATAL','log-fatal'] };

const rosoutSub = new ROSLIB.Topic({
  ros,
  name: '/rosout',
  messageType: 'rcl_interfaces/msg/Log',
});

rosoutSub.subscribe((msg) => {
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
  return s.replace(/&/g,'&amp;').replace(/</g,'&lt;').replace(/>/g,'&gt;');
}
