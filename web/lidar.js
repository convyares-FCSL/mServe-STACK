// ── ROS connection ─────────────────────────────────────────────────────────────

const ros = new ROSLIB.Ros({ url: `ws://${window.location.hostname}:9090` });

const elStatus    = document.getElementById('connection-status');
const elNodeState = document.getElementById('node-state');

ros.on('connection', () => {
  elStatus.textContent = 'ROS Connected';
  elStatus.className = 'status connected';
  refreshLidarState();
  refreshSlamState();
});
ros.on('error',  () => { elStatus.textContent = 'ROS Error';        elStatus.className = 'status disconnected'; });
ros.on('close',  () => { elStatus.textContent = 'ROS Disconnected'; elStatus.className = 'status disconnected'; });

// ── Lifecycle (generalized over node name — used for both mserve_lidar and
// the separate slam_toolbox process below) ─────────────────────────────────

const TRANSITIONS = { configure: 1, activate: 3, deactivate: 4, cleanup: 2 };

function changeState(nodeName, transitionId, onDone) {
  new ROSLIB.Service({
    ros,
    name: `/${nodeName}/change_state`,
    serviceType: 'lifecycle_msgs/srv/ChangeState',
  }).callService(
    new ROSLIB.ServiceRequest({ transition: { id: transitionId, label: '' } }),
    () => setTimeout(onDone, 400),
    (err) => console.error(`${nodeName} change_state error`, err),
  );
}

function refreshNodeState(nodeName, elState) {
  new ROSLIB.Service({
    ros,
    name: `/${nodeName}/get_state`,
    serviceType: 'lifecycle_msgs/srv/GetState',
  }).callService(new ROSLIB.ServiceRequest({}), (res) => {
    const label = res?.current_state?.label ?? 'unavailable';
    elState.textContent = label;
    elState.className = `state-label state-${label}`;
  }, () => {
    elState.textContent = 'unavailable';
    elState.className = 'state-label state-unavailable';
  });
}

function refreshLidarState() { refreshNodeState('mserve_lidar', elNodeState); }

document.getElementById('btn-configure').addEventListener('click',  () => changeState('mserve_lidar', TRANSITIONS.configure, refreshLidarState));
document.getElementById('btn-activate').addEventListener('click',   () => changeState('mserve_lidar', TRANSITIONS.activate, refreshLidarState));
document.getElementById('btn-deactivate').addEventListener('click', () => changeState('mserve_lidar', TRANSITIONS.deactivate, refreshLidarState));
document.getElementById('btn-cleanup').addEventListener('click',    () => changeState('mserve_lidar', TRANSITIONS.cleanup, refreshLidarState));

setInterval(refreshLidarState, 4000);

// ── SLAM Toolbox ─────────────────────────────────────────────────────────────

const elSlamNodeState = document.getElementById('slam-node-state');
const elSlamMsg       = document.getElementById('slam-msg');

function refreshSlamState() { refreshNodeState('slam_toolbox', elSlamNodeState); }

document.getElementById('slam-btn-configure').addEventListener('click',  () => changeState('slam_toolbox', TRANSITIONS.configure, refreshSlamState));
document.getElementById('slam-btn-activate').addEventListener('click',   () => changeState('slam_toolbox', TRANSITIONS.activate, refreshSlamState));
document.getElementById('slam-btn-deactivate').addEventListener('click', () => changeState('slam_toolbox', TRANSITIONS.deactivate, refreshSlamState));
document.getElementById('slam-btn-cleanup').addEventListener('click',    () => changeState('slam_toolbox', TRANSITIONS.cleanup, refreshSlamState));

setInterval(refreshSlamState, 4000);

document.getElementById('slam-btn-reset').addEventListener('click', () => {
  if (!confirm('Reset the map? This clears the current map and pose graph — cannot be undone.')) return;
  elSlamMsg.textContent = 'Resetting…';
  new ROSLIB.Service({
    ros,
    name: '/slam_toolbox/reset',
    serviceType: 'slam_toolbox/srv/Reset',
  }).callService(new ROSLIB.ServiceRequest({ pause_new_measurements: false }), () => {
    elSlamMsg.textContent = 'Map reset — /map will clear once the next scan is processed.';
  }, (err) => { elSlamMsg.textContent = `Error: ${err}`; });
});

document.getElementById('slam-btn-save-map').addEventListener('click', () => {
  const name = document.getElementById('slam-map-name').value.trim() || 'map';
  elSlamMsg.textContent = `Saving as "${name}"…`;
  new ROSLIB.Service({
    ros,
    name: '/slam_toolbox/save_map',
    serviceType: 'slam_toolbox/srv/SaveMap',
  }).callService(new ROSLIB.ServiceRequest({ name: { data: name } }), (res) => {
    elSlamMsg.textContent = res?.result === 0
      ? `Saved "${name}".`
      : `Save returned code ${res?.result} — check slam_toolbox's own log.`;
  }, (err) => { elSlamMsg.textContent = `Error: ${err}`; });
});

new ROSLIB.Topic({
  ros,
  name: '/map_metadata',
  messageType: 'nav_msgs/msg/MapMetaData',
}).subscribe((msg) => {
  document.getElementById('slam-map-size').textContent = `${msg.width} × ${msg.height}`;
  document.getElementById('slam-map-resolution').textContent = msg.resolution.toFixed(3);
});

// ── Parameters ───────────────────────────────────────────────────────────────

// rcl_interfaces/msg/ParameterType constants
const PARAM_TYPE_BOOL   = 1;
const PARAM_TYPE_INTEGER = 2;
const PARAM_TYPE_DOUBLE  = 3;
const PARAM_TYPE_STRING  = 4;

const listParamsSvc = new ROSLIB.Service({
  ros,
  name: '/mserve_lidar/list_parameters',
  serviceType: 'rcl_interfaces/srv/ListParameters',
});
const getParamsSvc = new ROSLIB.Service({
  ros,
  name: '/mserve_lidar/get_parameters',
  serviceType: 'rcl_interfaces/srv/GetParameters',
});
const setParamsSvc = new ROSLIB.Service({
  ros,
  name: '/mserve_lidar/set_parameters',
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
    elParamsMsg.textContent = 'Node must be unconfigured to apply device/baudrate changes.';
    return;
  }
  applyParameters();
});

// ── Scan plot + info ────────────────────────────────────────────────────────────

const elCanvas      = document.getElementById('scan-canvas');
const elPlaceholder = document.getElementById('scan-placeholder');
const elZoomLabel   = document.getElementById('zoom-label');
const ctx = elCanvas.getContext('2d');

let lastScanStamp = null;
let lastScanMsg   = null;

// ── Zoom ─────────────────────────────────────────────────────────────────────

const ZOOM_MIN_M = 1, ZOOM_MAX_M = 15;
let viewRangeM = 6;

function resizeCanvas() {
  const rect = elCanvas.parentElement.getBoundingClientRect();
  elCanvas.width = rect.width;
  elCanvas.height = 320;
  if (lastScanMsg) drawLaserScan(ctx, elCanvas, lastScanMsg, viewRangeM);
}
window.addEventListener('resize', resizeCanvas);
resizeCanvas();

function setZoom(rangeM) {
  viewRangeM = Math.min(ZOOM_MAX_M, Math.max(ZOOM_MIN_M, rangeM));
  elZoomLabel.textContent = `${viewRangeM.toFixed(1)} m`;
  if (lastScanMsg) drawLaserScan(ctx, elCanvas, lastScanMsg, viewRangeM);
}

document.getElementById('btn-zoom-in').addEventListener('click',  () => setZoom(viewRangeM * 0.8));
document.getElementById('btn-zoom-out').addEventListener('click', () => setZoom(viewRangeM * 1.25));
elCanvas.addEventListener('wheel', (e) => {
  e.preventDefault();
  setZoom(viewRangeM * (e.deltaY > 0 ? 1.1 : 0.9));
}, { passive: false });

setZoom(viewRangeM);

new ROSLIB.Topic({
  ros,
  name: '/scan',
  messageType: 'sensor_msgs/msg/LaserScan',
}).subscribe((msg) => {
  elPlaceholder.style.display = 'none';
  document.getElementById('info-placeholder').style.display = 'none';

  lastScanMsg = msg;
  drawLaserScan(ctx, elCanvas, msg, viewRangeM);

  const finiteRanges = msg.ranges.filter(r => Number.isFinite(r));
  const rMin = finiteRanges.length ? Math.min(...finiteRanges) : NaN;
  const rMax = finiteRanges.length ? Math.max(...finiteRanges) : NaN;

  document.getElementById('info-points').textContent = finiteRanges.length;
  document.getElementById('info-range').textContent =
    finiteRanges.length ? `${rMin.toFixed(2)} / ${rMax.toFixed(2)}` : '—';
  document.getElementById('info-frame-id').textContent = msg.header.frame_id || '—';

  const stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9;
  if (lastScanStamp !== null) {
    const dt = stamp - lastScanStamp;
    if (dt > 0) document.getElementById('info-rate').textContent = (1 / dt).toFixed(1);
  }
  lastScanStamp = stamp;
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
  if (!chkAllNodes.checked && !nodeName.includes('mserve_lidar')) return;

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
