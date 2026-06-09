// ── ROS connection ────────────────────────────────────────────────────────────

const ros = new ROSLIB.Ros({ url: `ws://${window.location.hostname}:9090` });

const elStatus   = document.getElementById('connection-status');
const elNodeState = document.getElementById('node-state');
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
  serviceType: 'mserve_interfaces/srv/DriveChainCmd',
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
  messageType: 'mserve_interfaces/msg/DriveStatus',
});

statusSub.subscribe((msg) => {
  const s = msg.status ?? '—';
  elDriveStatus.textContent = s;
  elDriveStatus.className = `service-status ${s.replace('_', '-')}`;
});

// ── Wheel feedback subscription ───────────────────────────────────────────────

const feedbackSub = new ROSLIB.Topic({
  ros,
  name: '/mserve_drivechain/wheel_feedback',
  messageType: 'mserve_interfaces/msg/WheelFeedback',
});

const RAD_TO_DEG = 180 / Math.PI;

feedbackSub.subscribe((msg) => {
  document.getElementById('fb-left-vel').textContent  = msg.left_velocity.toFixed(2);
  document.getElementById('fb-right-vel').textContent = msg.right_velocity.toFixed(2);
  document.getElementById('fb-left-pos').textContent  = (msg.left_position  * RAD_TO_DEG).toFixed(1);
  document.getElementById('fb-right-pos').textContent = (msg.right_position * RAD_TO_DEG).toFixed(1);
});

// ── cmd_vel publisher ─────────────────────────────────────────────────────────

const cmdVelTopic = new ROSLIB.Topic({
  ros,
  name: '/cmd_vel',
  messageType: 'geometry_msgs/msg/Twist',
});

function publishTwist(linear, angular) {
  cmdVelTopic.publish(new ROSLIB.Message({
    linear:  { x: linear,  y: 0, z: 0 },
    angular: { x: 0,       y: 0, z: angular },
  }));
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
  forward: () => publishTwist( getLinear(), 0),
  back:    () => publishTwist(-getLinear(), 0),
  left:    () => publishTwist(0,  getAngular()),
  right:   () => publishTwist(0, -getAngular()),
  stop:    () => publishTwist(0, 0),
};

function startDriving(dir) {
  if (activeDir === dir) return;
  stopDriving();
  activeDir = dir;
  document.getElementById(`btn-${dir}`)?.classList.add('active');

  if (dir === 'stop') {
    publishTwist(0, 0);
    return;
  }
  DIRS[dir]();
  driveInterval = setInterval(DIRS[dir], 100);
}

function stopDriving() {
  if (driveInterval) { clearInterval(driveInterval); driveInterval = null; }
  if (activeDir && activeDir !== 'stop') {
    document.getElementById(`btn-${activeDir}`)?.classList.remove('active');
    publishTwist(0, 0);
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

// ── Periodic state refresh ────────────────────────────────────────────────────

setInterval(refreshNodeState, 4000);

// ── /rosout log panel ─────────────────────────────────────────────────────────

const logBox       = document.getElementById('log-box');
const chkAllNodes  = document.getElementById('chk-all-nodes');
const chkDebug     = document.getElementById('chk-debug');

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
  const d = new Date();
  const ts = `${String(d.getHours()).padStart(2,'0')}:${String(d.getMinutes()).padStart(2,'0')}:${String(d.getSeconds()).padStart(2,'0')}`;

  const line = document.createElement('div');
  line.className = levelClass;
  line.innerHTML =
    `<span class="log-ts">${ts}</span>` +
    `<span class="log-node">[${nodeName}]</span>` +
    `<span>[${levelLabel}] ${escHtml(msg.msg ?? '')}</span>`;
  logBox.appendChild(line);

  // keep max 500 lines
  while (logBox.children.length > 500) logBox.removeChild(logBox.firstChild);

  // auto-scroll if already near the bottom
  if (logBox.scrollHeight - logBox.scrollTop < logBox.clientHeight + 60) {
    logBox.scrollTop = logBox.scrollHeight;
  }
});

function escHtml(s) {
  return s.replace(/&/g,'&amp;').replace(/</g,'&lt;').replace(/>/g,'&gt;');
}
