const ros = new ROSLIB.Ros({
  url: `ws://${window.location.hostname}:9090`,
});

const connectionStatus = document.getElementById('connection-status');

const linearSlider = document.getElementById('linear-speed');
const angularSlider = document.getElementById('angular-speed');
const linearValue = document.getElementById('linear-value');
const angularValue = document.getElementById('angular-value');

const publishCmd = document.getElementById('publish-cmd');
const stopCmd = document.getElementById('stop-cmd');

function setConnected(connected) {
  connectionStatus.textContent = connected ? 'Connected' : 'Disconnected';
  connectionStatus.className = connected ? 'status connected' : 'status disconnected';
}

ros.on('connection', () => setConnected(true));
ros.on('error', () => setConnected(false));
ros.on('close', () => setConnected(false));

const cmdVelTopic = new ROSLIB.Topic({
  ros,
  name: '/cmd_vel',
  messageType: 'geometry_msgs/msg/Twist',
});

function getState(nodeName) {
  return new Promise((resolve, reject) => {
    const service = new ROSLIB.Service({
      ros,
      name: `/${nodeName}/get_state`,
      serviceType: 'lifecycle_msgs/srv/GetState',
    });

    const request = new ROSLIB.ServiceRequest({});

    service.callService(request, (result) => {
      if (result && result.current_state) {
        resolve(result.current_state.label);
      } else {
        reject(new Error('No state returned'));
      }
    }, (error) => reject(error));
  });
}

function changeState(nodeName, transitionId) {
  const service = new ROSLIB.Service({
    ros,
    name: `/${nodeName}/change_state`,
    serviceType: 'lifecycle_msgs/srv/ChangeState',
  });

  const request = new ROSLIB.ServiceRequest({
    transition: { id: transitionId, label: '' },
  });

  service.callService(request, (result) => {
    console.log(`${nodeName} transitioned:`, result);
    refreshStates();
  }, (error) => {
    console.error('Change state error', error);
  });
}

function applyState(prefix, nodeName, state) {
  const el = document.getElementById(`${prefix}-state`);
  el.textContent = state || 'unavailable';
  el.className = `state-label state-${state || 'unavailable'}`;
  nodeStates[nodeName] = state || null;

  const dead = state === 'finalized' || state === 'errorprocessing' || !state;
  ['configure', 'activate', 'deactivate', 'cleanup', 'shutdown'].forEach((action) => {
    document.getElementById(`${prefix}-${action}`).disabled = dead;
  });
}

function refreshStates() {
  Object.entries(nodePrefix).forEach(([prefix, nodeName]) => {
    getState(nodeName)
      .then((state) => applyState(prefix, nodeName, state))
      .catch(() => applyState(prefix, nodeName, null));
  });
}

const transitions = {
  configure: 1,
  activate: 3,
  deactivate: 4,
  cleanup: 2,
};

const shutdownIds = { unconfigured: 5, inactive: 6, active: 7 };

const nodeStates = { mserve_base: null, mserve_drivechain: null, mserve_camera: null, mserve_lidar: null };

const nodePrefix = { base: 'mserve_base', drivechain: 'mserve_drivechain', camera: 'mserve_camera', lidar: 'mserve_lidar' };

Object.keys(nodePrefix).forEach((prefix) => {
  ['configure', 'activate', 'deactivate', 'cleanup'].forEach((action) => {
    document.getElementById(`${prefix}-${action}`).addEventListener('click', () => {
      changeState(nodePrefix[prefix], transitions[action]);
    });
  });

  document.getElementById(`${prefix}-shutdown`).addEventListener('click', () => {
    const nodeName = nodePrefix[prefix];
    const id = shutdownIds[nodeStates[nodeName]];
    if (id == null) {
      console.warn(`No shutdown transition for state: ${nodeStates[nodeName]}`);
      return;
    }
    changeState(nodeName, id);
  });
});

function publishTwist(linear, angular) {
  const twist = new ROSLIB.Message({
    linear: { x: linear, y: 0, z: 0 },
    angular: { x: 0, y: 0, z: angular },
  });
  cmdVelTopic.publish(twist);
}

linearSlider.addEventListener('input', () => {
  linearValue.textContent = Number(linearSlider.value).toFixed(2);
});

angularSlider.addEventListener('input', () => {
  angularValue.textContent = Number(angularSlider.value).toFixed(2);
});

publishCmd.addEventListener('click', () => {
  publishTwist(parseFloat(linearSlider.value), parseFloat(angularSlider.value));
});

stopCmd.addEventListener('click', () => {
  linearSlider.value = 0;
  angularSlider.value = 0;
  linearValue.textContent = '0.00';
  angularValue.textContent = '0.00';
  publishTwist(0, 0);
});

setInterval(refreshStates, 3000);
refreshStates();
