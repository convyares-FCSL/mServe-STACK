// ── ROS connection ───────────────────────────────────────────────────────

const ros = new ROSLIB.Ros({ url: `ws://${window.location.hostname}:9090` });

const elStatus = document.getElementById('connection-status');

ros.on('connection', () => {
  elStatus.textContent = 'ROS Connected';
  elStatus.className = 'status connected';
});
ros.on('error', () => { elStatus.textContent = 'ROS Error';        elStatus.className = 'status disconnected'; });
ros.on('close', () => { elStatus.textContent = 'ROS Disconnected'; elStatus.className = 'status disconnected'; });

// ── Parameters (reused pattern from joystick.js/lidar.js/camera.js) ─────

const PARAM_TYPE_BOOL   = 1;
const PARAM_TYPE_INTEGER = 2;
const PARAM_TYPE_DOUBLE  = 3;
const PARAM_TYPE_STRING  = 4;

const listParamsSvc = new ROSLIB.Service({
  ros, name: '/mserve_sensehat/list_parameters', serviceType: 'rcl_interfaces/srv/ListParameters',
});
const getParamsSvc = new ROSLIB.Service({
  ros, name: '/mserve_sensehat/get_parameters', serviceType: 'rcl_interfaces/srv/GetParameters',
});
const setParamsSvc = new ROSLIB.Service({
  ros, name: '/mserve_sensehat/set_parameters', serviceType: 'rcl_interfaces/srv/SetParameters',
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
document.getElementById('btn-params-apply').addEventListener('click', applyParameters);

// ── Online/offline toggle ────────────────────────────────────────────────

const elOnlineToggle = document.getElementById('btn-online-toggle');
let currentOnline = true;  // mirrors the node's own default until the first status message arrives

function renderOnlineToggle(online) {
  currentOnline = online;
  elOnlineToggle.textContent = online ? 'Online' : 'Offline';
  elOnlineToggle.className = online ? 'is-online' : 'is-offline';
}
renderOnlineToggle(true);

const setOnlineSvc = new ROSLIB.Service({
  ros, name: '/mserve_sensehat/set_online', serviceType: 'std_srvs/srv/SetBool',
});

elOnlineToggle.addEventListener('click', () => {
  const next = !currentOnline;
  setOnlineSvc.callService(
    new ROSLIB.ServiceRequest({ data: next }),
    () => renderOnlineToggle(next),
    (err) => console.error('set_online error', err),
  );
});

// ── Status (env, IMU, compass, joystick, availability) ──────────────────

const fmt = (v, digits = 2) => (typeof v === 'number' ? v.toFixed(digits) : '—');
const availText = (v) => (v ? 'Available' : 'Not detected');

const dpadEls = {
  up: document.getElementById('joy-up'),
  down: document.getElementById('joy-down'),
  left: document.getElementById('joy-left'),
  right: document.getElementById('joy-right'),
  center: document.getElementById('joy-center'),
};
const elCompassRing = document.getElementById('compass-ring');
const compassTickEls = [...document.querySelectorAll('#compass-ring .compass-tick')];
const elImuNotAvailable = document.getElementById('imu-not-available');

// Ring rotates -heading so true north stays correctly placed as the robot
// turns (a fixed pointer at the top of the dial then reads off the current
// heading) — see the CSS comment on .compass-ring for why, and
// web/sensehat.html's <p> for the phone-compass mismatch this fixes.
function setCompassHeading(headingDeg) {
  const heading = headingDeg ?? 0;
  elCompassRing.style.transform = `rotate(${-heading}deg)`;
  for (const el of compassTickEls) {
    el.style.transform = `${el.dataset.base} rotate(${heading}deg)`;
  }
}
setCompassHeading(0);

new ROSLIB.Topic({
  ros, name: '/mserve_sensehat/status', messageType: 'interfaces/msg/SensehatStatus',
}).subscribe((msg) => {
  renderOnlineToggle(msg.online);

  document.getElementById('env-temperature').textContent = fmt(msg.temperature_c, 1);
  document.getElementById('env-pressure').textContent = fmt(msg.pressure_hpa, 1);
  document.getElementById('env-humidity').textContent = fmt(msg.humidity_percent, 1);

  elImuNotAvailable.style.display = msg.imu_available ? 'none' : 'block';
  document.getElementById('accel-x').textContent = fmt(msg.accel_g?.x);
  document.getElementById('accel-y').textContent = fmt(msg.accel_g?.y);
  document.getElementById('accel-z').textContent = fmt(msg.accel_g?.z);
  document.getElementById('gyro-x').textContent = fmt(msg.gyro_dps?.x, 1);
  document.getElementById('gyro-y').textContent = fmt(msg.gyro_dps?.y, 1);
  document.getElementById('gyro-z').textContent = fmt(msg.gyro_dps?.z, 1);
  document.getElementById('mag-x').textContent = fmt(msg.mag_ut?.x, 1);
  document.getElementById('mag-y').textContent = fmt(msg.mag_ut?.y, 1);
  document.getElementById('mag-z').textContent = fmt(msg.mag_ut?.z, 1);

  document.getElementById('compass-heading').textContent = msg.imu_available ? `${fmt(msg.heading_deg, 0)}°` : '—';
  setCompassHeading(msg.heading_deg);
  document.getElementById('compass-uncalibrated-warning').style.display =
    (msg.imu_available && !msg.compass_calibrated) ? 'block' : 'none';

  dpadEls.up.classList.toggle('lit', !!msg.joy_up);
  dpadEls.down.classList.toggle('lit', !!msg.joy_down);
  dpadEls.left.classList.toggle('lit', !!msg.joy_left);
  dpadEls.right.classList.toggle('lit', !!msg.joy_right);
  dpadEls.center.classList.toggle('lit', !!msg.joy_center);

  document.getElementById('avail-imu').textContent = availText(msg.imu_available);
  document.getElementById('avail-pressure').textContent = availText(msg.pressure_available);
  document.getElementById('avail-humidity').textContent = availText(msg.humidity_available);
  document.getElementById('avail-led').textContent = availText(msg.led_available);
  document.getElementById('avail-joystick').textContent = availText(msg.joystick_available);
  document.getElementById('avail-compass-cal').textContent = availText(msg.compass_calibrated);
  document.getElementById('avail-accel-cal').textContent = availText(msg.accel_calibrated);
});

// ── LED matrix status icon (derived the same way the node itself renders
// it — from drive_status, not re-read from hardware) ─────────────────────

new ROSLIB.Topic({
  ros, name: '/mserve_drivechain/drive_status', messageType: 'interfaces/msg/DriveStatus',
}).subscribe((msg) => {
  const connected = (msg.status || '').indexOf('connected') === 0;
  document.getElementById('led-status-icon').textContent = connected ? '🟢 O (connected)' : '🔴 X (disconnected)';
});
