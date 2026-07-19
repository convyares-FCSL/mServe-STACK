// ── ROS connection ─────────────────────────────────────────────────────────────

const ros = new ROSLIB.Ros({ url: `ws://${window.location.hostname}:9090` });

const elStatus = document.getElementById('connection-status');

ros.on('connection', () => {
  elStatus.textContent = 'ROS Connected';
  elStatus.className = 'status connected';
});
ros.on('error', () => { elStatus.textContent = 'ROS Error';        elStatus.className = 'status disconnected'; });
ros.on('close', () => { elStatus.textContent = 'ROS Disconnected'; elStatus.className = 'status disconnected'; });

// ── Parameters (reused pattern from lidar.js/camera.js/drivechain.js) ───────────

// rcl_interfaces/msg/ParameterType constants
const PARAM_TYPE_BOOL   = 1;
const PARAM_TYPE_INTEGER = 2;
const PARAM_TYPE_DOUBLE  = 3;
const PARAM_TYPE_STRING  = 4;

const listParamsSvc = new ROSLIB.Service({
  ros,
  name: '/mserve_joystick/list_parameters',
  serviceType: 'rcl_interfaces/srv/ListParameters',
});
const getParamsSvc = new ROSLIB.Service({
  ros,
  name: '/mserve_joystick/get_parameters',
  serviceType: 'rcl_interfaces/srv/GetParameters',
});
const setParamsSvc = new ROSLIB.Service({
  ros,
  name: '/mserve_joystick/set_parameters',
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
// No lifecycle-state guard on apply (unlike lidar.js/camera.js) — mserve_joystick
// is a plain node, not lifecycle-managed, so there's no unconfigured/active
// gating on any of its parameters.
document.getElementById('btn-params-apply').addEventListener('click', applyParameters);

// ── Status (controller profile, speed/angular scale, online) ────────────────────

const elOnlineToggle = document.getElementById('btn-online-toggle');
let currentOnline = true;  // mirrors the node's own default until the first status message arrives

function renderOnlineToggle(online) {
  currentOnline = online;
  elOnlineToggle.textContent = online ? 'Online' : 'Offline';
  elOnlineToggle.className = online ? 'is-online' : 'is-offline';
}
renderOnlineToggle(true);

new ROSLIB.Topic({
  ros,
  name: '/mserve_joystick/status',
  messageType: 'interfaces/msg/JoystickStatus',
}).subscribe((msg) => {
  document.getElementById('controller-profile').textContent = msg.controller_profile ?? '—';
  document.getElementById('speed-scale').textContent = `${msg.speed_scale.toFixed(2)} m/s`;
  document.getElementById('angular-scale').textContent = `${msg.angular_scale.toFixed(2)} rad/s`;
  renderOnlineToggle(msg.online ?? true);
});

const setOnlineSvc = new ROSLIB.Service({
  ros,
  name: '/mserve_joystick/set_online',
  serviceType: 'std_srvs/srv/SetBool',
});

elOnlineToggle.addEventListener('click', () => {
  const next = !currentOnline;
  setOnlineSvc.callService(
    new ROSLIB.ServiceRequest({ data: next }),
    () => renderOnlineToggle(next),
    (err) => console.error('set_online error', err),
  );
});

// ── Live /cmd_vel readout ─────────────────────────────────────────────────────

new ROSLIB.Topic({
  ros,
  name: '/cmd_vel',
  messageType: 'geometry_msgs/msg/Twist',
}).subscribe((msg) => {
  document.getElementById('cmd-linear').textContent = msg.linear.x.toFixed(2);
  document.getElementById('cmd-angular').textContent = msg.angular.z.toFixed(2);
});

// ── Raw /joy — sticks, triggers, D-pad, buttons ──────────────────────────────
//
// Index -> label matches interfaces/config/joystick_params.yaml's
// pi_controller profile. Purely cosmetic (labels below) — if this is ever
// pointed at a different controller_profile, these will just show the
// wrong labels, not break anything.
//
// Index 12 deliberately omitted: confirmed on real hardware to report as
// analog, not a digital button — see joystick_params.yaml's comment.

const BUTTON_LABELS = {
  0: 'Cross', 1: 'Circle', 3: 'Square', 4: 'Triangle',
  6: 'L1', 7: 'R1', 8: 'L2', 9: 'R2',
  10: 'Select', 11: 'Start', 13: 'L3', 14: 'R3',
};

const buttonsGrid = document.getElementById('buttons-grid');
const buttonEls = {};
for (const [index, label] of Object.entries(BUTTON_LABELS)) {
  const el = document.createElement('div');
  el.className = 'button-indicator';
  el.textContent = label;
  buttonsGrid.appendChild(el);
  buttonEls[index] = el;
}

const elLeftDot  = document.getElementById('left-stick-dot');
const elRightDot = document.getElementById('right-stick-dot');
const elL2Fill = document.getElementById('l2-fill');
const elR2Fill = document.getElementById('r2-fill');
const elNoJoyNotice = document.getElementById('no-joy-notice');
const dpadEls = {
  u: document.getElementById('dpad-u'), d: document.getElementById('dpad-d'),
  l: document.getElementById('dpad-l'), r: document.getElementById('dpad-r'),
};

const STICK_RADIUS_PX = 55;

function setStickDot(el, x, y) {
  // Screen Y grows downward; stick "up" is a positive raw axis value
  // (confirmed on real hardware) — negate so the dot moves up when the
  // stick is pushed up. X is also negated: confirmed on real hardware that
  // pushing right yields a NEGATIVE raw axes[0]/axes[2] (this pad's raw
  // convention, not a Y-axis-only quirk) — negate so the dot moves right
  // when the stick is pushed right.
  el.style.transform = `translate(${-x * STICK_RADIUS_PX - 8}px, ${-y * STICK_RADIUS_PX - 8}px)`;
}

let joyReceived = false;

new ROSLIB.Topic({
  ros,
  name: '/joy',
  messageType: 'sensor_msgs/msg/Joy',
}).subscribe((msg) => {
  if (!joyReceived) {
    joyReceived = true;
    elNoJoyNotice.style.display = 'none';
  }

  const axes = msg.axes ?? [];
  const buttons = msg.buttons ?? [];

  setStickDot(elLeftDot, axes[0] ?? 0, axes[1] ?? 0);
  setStickDot(elRightDot, axes[2] ?? 0, axes[3] ?? 0);

  // Rest = +1.0, full squeeze = -1.0 (confirmed on real hardware).
  const triggerFill = (v) => `${((1 - (v ?? 1)) / 2 * 100).toFixed(0)}%`;
  elR2Fill.style.height = triggerFill(axes[4]);
  elL2Fill.style.height = triggerFill(axes[5]);

  // Left/right confirmed swapped from the naive raw-sign guess on real
  // hardware — same raw-convention story as the sticks above.
  const dpadX = axes[6] ?? 0, dpadY = axes[7] ?? 0;
  dpadEls.l.classList.toggle('lit', dpadX > 0.5);
  dpadEls.r.classList.toggle('lit', dpadX < -0.5);
  dpadEls.u.classList.toggle('lit', dpadY > 0.5);
  dpadEls.d.classList.toggle('lit', dpadY < -0.5);

  for (const [index, el] of Object.entries(buttonEls)) {
    el.classList.toggle('lit', buttons[index] === 1);
  }
});
