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
    name: '/mserve_camera/change_state',
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
    name: '/mserve_camera/get_state',
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

// ── Parameters ───────────────────────────────────────────────────────────────

// rcl_interfaces/msg/ParameterType constants
const PARAM_TYPE_BOOL   = 1;
const PARAM_TYPE_INTEGER = 2;
const PARAM_TYPE_DOUBLE  = 3;
const PARAM_TYPE_STRING  = 4;

const listParamsSvc = new ROSLIB.Service({
  ros,
  name: '/mserve_camera/list_parameters',
  serviceType: 'rcl_interfaces/srv/ListParameters',
});
const getParamsSvc = new ROSLIB.Service({
  ros,
  name: '/mserve_camera/get_parameters',
  serviceType: 'rcl_interfaces/srv/GetParameters',
});
const setParamsSvc = new ROSLIB.Service({
  ros,
  name: '/mserve_camera/set_parameters',
  serviceType: 'rcl_interfaces/srv/SetParameters',
});

const elParamsBody = document.getElementById('params-table-body');
const elParamsMsg  = document.getElementById('params-msg');

function paramValueToText(value) {
  switch (value.type) {
    case PARAM_TYPE_BOOL:    return String(value.bool_value);
    case PARAM_TYPE_INTEGER: return String(value.integer_value);
    case PARAM_TYPE_DOUBLE:  return String(value.double_value);
    case PARAM_TYPE_STRING:  return value.string_value;
    default:                 return '';
  }
}

function textToParamValue(type, text) {
  switch (type) {
    case PARAM_TYPE_BOOL:    return { type, bool_value: text };
    case PARAM_TYPE_INTEGER: return { type, integer_value: parseInt(text, 10) || 0 };
    case PARAM_TYPE_DOUBLE:  return { type, double_value: parseFloat(text) || 0 };
    case PARAM_TYPE_STRING:  return { type, string_value: text };
    default:                 return { type: 0 };
  }
}

function loadParameters() {
  elParamsMsg.textContent = 'Loading…';
  listParamsSvc.callService(new ROSLIB.ServiceRequest({ prefixes: [], depth: 0 }), (listRes) => {
    const names = listRes?.result?.names ?? [];
    if (names.length === 0) {
      elParamsBody.innerHTML = '<tr><td colspan="2" class="motor-cmd-empty">No parameters reported.</td></tr>';
      elParamsMsg.textContent = '';
      return;
    }

    getParamsSvc.callService(new ROSLIB.ServiceRequest({ names }), (getRes) => {
      elParamsBody.innerHTML = '';
      names.forEach((name, i) => {
        const value = getRes.values[i];
        const row = document.createElement('tr');
        row.dataset.paramName = name;
        row.dataset.paramType = value.type;

        const nameCell = document.createElement('td');
        nameCell.className = 'param-name';
        nameCell.textContent = name;

        const valueCell = document.createElement('td');
        const input = document.createElement('input');
        input.className = 'param-input';
        if (value.type === PARAM_TYPE_BOOL) {
          input.type = 'checkbox';
          input.checked = value.bool_value;
        } else {
          input.type = 'text';
          input.value = paramValueToText(value);
        }
        valueCell.appendChild(input);

        row.append(nameCell, valueCell);
        elParamsBody.appendChild(row);
      });
      elParamsMsg.textContent = `Loaded ${names.length} parameters.`;
    }, (err) => { elParamsMsg.textContent = `Error: ${err}`; });
  }, (err) => { elParamsMsg.textContent = `Error: ${err}`; });
}

function applyParameters() {
  const rows = [...elParamsBody.querySelectorAll('tr[data-param-name]')];
  const parameters = rows.map((row) => {
    const name = row.dataset.paramName;
    const type = parseInt(row.dataset.paramType, 10);
    const input = row.querySelector('.param-input');
    const text = (type === PARAM_TYPE_BOOL) ? String(input.checked) : input.value;
    return { name, value: textToParamValue(type, text) };
  });

  elParamsMsg.textContent = 'Applying…';
  setParamsSvc.callService(new ROSLIB.ServiceRequest({ parameters }), (res) => {
    const results = res?.results ?? [];
    const failed = results.filter(r => !r.successful);
    if (failed.length === 0) {
      elParamsMsg.textContent = `Applied ${results.length} parameters.`;
    } else {
      elParamsMsg.textContent = `${failed.length} failed: ${failed.map(r => r.reason).join('; ')}`;
    }
  }, (err) => { elParamsMsg.textContent = `Error: ${err}`; });
}

document.getElementById('btn-params-refresh').addEventListener('click', loadParameters);
document.getElementById('btn-params-apply').addEventListener('click', () => {
  if (elNodeState.textContent !== 'unconfigured') {
    elParamsMsg.textContent = 'Node must be unconfigured to apply device/width/height changes.';
    return;
  }
  applyParameters();
});

// ── Image (via web_video_server MJPEG transcode) ──────────────────────────────

const elImage        = document.getElementById('camera-image');
const elImagePlaceholder = document.getElementById('camera-image-placeholder');

const STREAM_URL = `http://${window.location.hostname}:8080/stream?topic=/camera/image_raw&type=mjpeg`;

elImage.addEventListener('error', () => {
  elImage.style.display = 'none';
  elImagePlaceholder.style.display = 'block';
  elImagePlaceholder.textContent = 'No stream — is the node active and web_video_server running?';
  // Retry — the node may just not be active yet.
  setTimeout(startImageStream, 3000);
});
elImage.addEventListener('load', () => {
  elImage.style.display = 'block';
  elImagePlaceholder.style.display = 'none';
});

function startImageStream() {
  elImage.src = `${STREAM_URL}&_ts=${Date.now()}`;
}
startImageStream();

// ── Camera info subscription ──────────────────────────────────────────────────

new ROSLIB.Topic({
  ros,
  name: '/camera/camera_info',
  messageType: 'sensor_msgs/msg/CameraInfo',
}).subscribe((msg) => {
  document.getElementById('info-placeholder').style.display = 'none';
  document.getElementById('info-resolution').textContent = `${msg.width} × ${msg.height}`;
  document.getElementById('info-frame-id').textContent = msg.header.frame_id || '—';
  document.getElementById('info-distortion').textContent =
    msg.distortion_model ? msg.distortion_model : 'uncalibrated (no distortion model set)';
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
  if (!chkAllNodes.checked && !nodeName.includes('mserve_camera') && !nodeName.includes('v4l2_camera')) return;

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
