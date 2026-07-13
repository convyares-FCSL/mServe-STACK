// ── ROS connection ─────────────────────────────────────────────────────────────

const ros = new ROSLIB.Ros({ url: `ws://${window.location.hostname}:9090` });

const elStatus    = document.getElementById('connection-status');
const elNodeState = document.getElementById('node-state');

ros.on('connection', () => {
  elStatus.textContent = 'ROS Connected';
  elStatus.className = 'status connected';
  refreshNodeState();
});
ros.on('error',  () => { elStatus.textContent = 'ROS Error';        elStatus.className = 'status disconnected'; });
ros.on('close',  () => { elStatus.textContent = 'ROS Disconnected'; elStatus.className = 'status disconnected'; });

// ── Lifecycle ──────────────────────────────────────────────────────────────────

const TRANSITIONS = { configure: 1, activate: 3, deactivate: 4, cleanup: 2 };

function changeState(transitionId) {
  new ROSLIB.Service({
    ros,
    name: '/mserve_base/change_state',
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
    name: '/mserve_base/get_state',
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
document.getElementById('btn-cleanup').addEventListener('click',    () => changeState(TRANSITIONS.cleanup));

setInterval(refreshNodeState, 4000);

// ── Drive controls ─────────────────────────────────────────────────────────────

const cmdVelTopic = new ROSLIB.Topic({
  ros,
  name: '/cmd_vel',
  messageType: 'geometry_msgs/msg/Twist',
});

const speedLinearSlider  = document.getElementById('speed-linear');
const speedAngularSlider = document.getElementById('speed-angular');
const speedLinearVal     = document.getElementById('speed-linear-val');
const speedAngularVal    = document.getElementById('speed-angular-val');

speedLinearSlider.addEventListener('input', () => {
  speedLinearVal.textContent = `${parseFloat(speedLinearSlider.value).toFixed(2)} m/s`;
});
speedAngularSlider.addEventListener('input', () => {
  speedAngularVal.textContent = `${parseFloat(speedAngularSlider.value).toFixed(2)} rad/s`;
});

function publishTwist(linear, angular) {
  cmdVelTopic.publish(new ROSLIB.Message({
    linear:  { x: linear, y: 0, z: 0 },
    angular: { x: 0, y: 0, z: angular },
  }));
}

let driveInterval = null;
let currentTwist  = { linear: 0, angular: 0 };

function startDrive(linear, angular) {
  currentTwist = { linear, angular };
  publishTwist(linear, angular);
  if (driveInterval) clearInterval(driveInterval);
  driveInterval = setInterval(() => publishTwist(currentTwist.linear, currentTwist.angular), 100);
}

function stopDrive() {
  if (driveInterval) { clearInterval(driveInterval); driveInterval = null; }
  publishTwist(0, 0);
}

// Bind a D-pad button to start driving on press and stop on release, covering
// both mouse and touch input.
function bindHold(buttonId, getLinear, getAngular) {
  const btn = document.getElementById(buttonId);
  const press = (e) => { e.preventDefault(); startDrive(getLinear(), getAngular()); };
  const release = (e) => { e.preventDefault(); stopDrive(); };
  btn.addEventListener('mousedown', press);
  btn.addEventListener('touchstart', press, { passive: false });
  ['mouseup', 'mouseleave', 'touchend', 'touchcancel'].forEach((evt) => btn.addEventListener(evt, release));
}

bindHold('btn-fwd',   () =>  parseFloat(speedLinearSlider.value),  () => 0);
bindHold('btn-back',  () => -parseFloat(speedLinearSlider.value),  () => 0);
bindHold('btn-left',  () => 0, () =>  parseFloat(speedAngularSlider.value));
bindHold('btn-right', () => 0, () => -parseFloat(speedAngularSlider.value));

document.getElementById('btn-stop').addEventListener('click', stopDrive);

// Safety: stop driving if the page loses focus while a button is held.
window.addEventListener('blur', stopDrive);

// ── Image (via web_video_server MJPEG transcode) ──────────────────────────────

const elCameraImage    = document.getElementById('camera-image');
const elCameraPlaceholder = document.getElementById('camera-image-placeholder');

const STREAM_URL = `http://${window.location.hostname}:8080/stream?topic=/camera/image_raw&type=mjpeg`;

elCameraImage.addEventListener('error', () => {
  elCameraImage.style.display = 'none';
  elCameraPlaceholder.style.display = 'block';
  setTimeout(startImageStream, 3000);
});
elCameraImage.addEventListener('load', () => {
  elCameraImage.style.display = 'block';
  elCameraPlaceholder.style.display = 'none';
});

function startImageStream() {
  elCameraImage.src = `${STREAM_URL}&_ts=${Date.now()}`;
}
startImageStream();

// ── DriveChain connect (opens the UART port — activating the node alone doesn't) ──

const connectSvc = new ROSLIB.Service({
  ros,
  name: '/mserve_drivechain/connect',
  serviceType: 'std_srvs/srv/Trigger',
});

document.getElementById('btn-connect').addEventListener('click', () => {
  connectSvc.callService(new ROSLIB.ServiceRequest({}), () => {}, (err) => console.error('connect error', err));
});

new ROSLIB.Topic({
  ros,
  name: '/mserve_drivechain/drive_status',
  messageType: 'interfaces/msg/DriveStatus',
}).subscribe((msg) => {
  const s = msg.status ?? '—';
  const elDriveStatus = document.getElementById('drive-status');
  elDriveStatus.textContent = s;
  elDriveStatus.className = `service-status ${s.replace('_', '-')}`;
});

// ── Scan (zoomable — see lidar_view.js for the shared renderer) ────────────────

const elScanCanvas      = document.getElementById('scan-canvas');
const elScanPlaceholder = document.getElementById('scan-placeholder');
const elZoomLabel       = document.getElementById('zoom-label');
const scanCtx = elScanCanvas.getContext('2d');

let lastScanMsg = null;
const ZOOM_MIN_M = 1, ZOOM_MAX_M = 15;
let scanViewRangeM = 6;

function resizeScanCanvas() {
  const rect = elScanCanvas.parentElement.getBoundingClientRect();
  elScanCanvas.width = rect.width;
  elScanCanvas.height = 320;
  if (lastScanMsg) drawLaserScan(scanCtx, elScanCanvas, lastScanMsg, scanViewRangeM);
}
window.addEventListener('resize', resizeScanCanvas);
resizeScanCanvas();

function setScanZoom(rangeM) {
  scanViewRangeM = Math.min(ZOOM_MAX_M, Math.max(ZOOM_MIN_M, rangeM));
  elZoomLabel.textContent = `${scanViewRangeM.toFixed(1)} m`;
  if (lastScanMsg) drawLaserScan(scanCtx, elScanCanvas, lastScanMsg, scanViewRangeM);
}

document.getElementById('btn-zoom-in').addEventListener('click',  () => setScanZoom(scanViewRangeM * 0.8));
document.getElementById('btn-zoom-out').addEventListener('click', () => setScanZoom(scanViewRangeM * 1.25));
elScanCanvas.addEventListener('wheel', (e) => {
  e.preventDefault();
  setScanZoom(scanViewRangeM * (e.deltaY > 0 ? 1.1 : 0.9));
}, { passive: false });

setScanZoom(scanViewRangeM);

new ROSLIB.Topic({
  ros,
  name: '/scan',
  messageType: 'sensor_msgs/msg/LaserScan',
}).subscribe((msg) => {
  elScanPlaceholder.style.display = 'none';
  lastScanMsg = msg;
  drawLaserScan(scanCtx, elScanCanvas, msg, scanViewRangeM);
});

// ── Status subscriptions ─────────────────────────────────────────────────────

new ROSLIB.Topic({
  ros,
  name: '/mserve/cmd_vel_safe',
  messageType: 'geometry_msgs/msg/Twist',
}).subscribe((msg) => {
  document.getElementById('safe-linear').textContent  = msg.linear.x.toFixed(2);
  document.getElementById('safe-angular').textContent = msg.angular.z.toFixed(2);
});

new ROSLIB.Topic({
  ros,
  name: '/mserve_base/base_status',
  messageType: 'interfaces/msg/DriveStatus',
}).subscribe((msg) => {
  const elBridgeStatus = document.getElementById('bridge-status');
  elBridgeStatus.textContent = msg.status ?? '—';
  elBridgeStatus.style.color = msg.board_alive ? 'var(--success)' : 'var(--danger)';
  document.getElementById('bridge-detail').textContent =
    `battery ${msg.battery_level.toFixed(1)} V`;
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
  if (!chkAllNodes.checked && !nodeName.includes('mserve_base')) return;

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
