import { keyboardController } from './KeyboardControl.js';

const MAX_BUFFERED_BYTES = 2 * 1024 * 1024;

/** Encode packed sensor bytes without exceeding JavaScript's argument limit. */
function typedArrayToBase64(array) {
  if (!array) return null;
  const bytes = new Uint8Array(array.buffer, array.byteOffset, array.byteLength);
  let binary = '';
  const chunkSize = 0x8000;
  for (let offset = 0; offset < bytes.length; offset += chunkSize) {
    binary += String.fromCharCode(...bytes.subarray(offset, offset + chunkSize));
  }
  return btoa(binary);
}

function finiteArray(values) {
  return Array.from(values ?? [], (value) => Number.isFinite(value) ? Number(value) : null);
}

/** Bidirectional browser transport for the host ROS 2 bridge. */
export class MujocoBridgeClient {
  /** Configure the single web physics connection to the host ROS bridge. */
  constructor(demo) {
    this.demo = demo;
    this.socket = null;
    this.reconnectTimer = null;
    this.disabled = new URLSearchParams(window.location.search).get('bridge') === 'off';
    const queryUrl = new URLSearchParams(window.location.search).get('bridgeUrl');
    this.url = queryUrl || `ws://${window.location.hostname || '127.0.0.1'}:8765`;
    this.status = {
      connected: false,
      stateFrames: 0,
      sensorFrames: 0,
      lastError: '',
      url: this.url
    };
    this.nextStateTimeMs = 0;
    this.nextCameraTimeMs = 0;
    this.lastPlanarTimestamp = -1;
    this.lastCloudTimestamp = -1;
    this.cameraIndex = 0;
  }

  start() {
    if (!this.disabled) this._connect();
  }

  /** Close transport and clear external velocity on shutdown. */
  stop() {
    this.disabled = true;
    clearTimeout(this.reconnectTimer);
    this.socket?.close(1000, 'client stop');
    keyboardController.setExternalControlEnabled(false);
  }

  /** Connect and announce the web backend; reconnect after ordinary connection loss. */
  _connect() {
    if (this.disabled || this.socket?.readyState === WebSocket.OPEN ||
        this.socket?.readyState === WebSocket.CONNECTING) return;
    try {
      this.socket = new WebSocket(this.url);
      this.socket.addEventListener('open', () => {
        this.status.connected = true;
        this.status.lastError = '';
        keyboardController.setExternalControlEnabled(true);
        this._send({
          type: 'hello',
          protocolVersion: 1,
          backend: 'web',
          environment: this.demo.params.environment,
          robot: this.demo.params.robot,
          visualMode: this.demo.sceneManager.getVisualMode()
        }, true);
      });
      this.socket.addEventListener('message', (event) => this._handleMessage(event.data));
      this.socket.addEventListener('error', () => {
        this.status.lastError = 'WebSocket connection failed';
      });
      this.socket.addEventListener('close', (event) => {
        this.status.connected = false;
        keyboardController.emergencyStop();
        keyboardController.setExternalControlEnabled(false);
        this.socket = null;
        if (!this.disabled && event.code !== 4001) {
          this.reconnectTimer = setTimeout(() => this._connect(), 1000);
        }
      });
    } catch (error) {
      this.status.lastError = String(error);
      this.reconnectTimer = setTimeout(() => this._connect(), 1000);
    }
  }

  _send(payload, priority = false) {
    if (this.socket?.readyState !== WebSocket.OPEN) return false;
    if (!priority && this.socket.bufferedAmount >= MAX_BUFFERED_BYTES) return false;
    const encoded = JSON.stringify(payload);
    if (!priority && this.socket.bufferedAmount + encoded.length > MAX_BUFFERED_BYTES) return false;
    this.socket.send(encoded);
    return true;
  }

  /** Acknowledge commands only after completion, returning policy errors to HTTP callers. */
  async _handleMessage(raw) {
    let message;
    try {
      message = JSON.parse(raw);
    } catch {
      this.status.lastError = 'Bridge sent invalid JSON';
      return;
    }
    if (!message || typeof message !== 'object') {
      this.status.lastError = 'Bridge command must be an object';
      return;
    }
    let accepted = true;
    let error = null;
    try {
      if (message.type === 'cmd_vel') {
        accepted = keyboardController.setTwist(message);
      } else if (message.type === 'policy') {
        accepted = await keyboardController.policyCommand(message);
      } else if (message.type === 'emergency_stop') {
        accepted = keyboardController.emergencyStop();
      } else if (message.type === 'reset') {
        accepted = keyboardController.resetRobot();
      } else if (message.type === 'ping') {
        this._send({ type: 'pong', sequence: message.sequence, simulationTime: Number(this.demo.data?.time ?? 0) }, true);
        return;
      } else {
        accepted = false;
      }
    } catch (failure) {
      accepted = false;
      error = String(failure.message ?? failure);
      this.status.lastError = error;
    }
    this._send({ type: 'command_ack', sequence: message.sequence ?? null, accepted: Boolean(accepted), error }, true);
    if (message.type === 'policy') this._publishState();
  }

  /** Publish state at 50 Hz and sensor samples at their configured rates. */
  update(timeMs) {
    if (!this.status.connected || !this.demo.model || !this.demo.data) return;
    if (timeMs >= this.nextStateTimeMs) {
      this._publishState();
      this.nextStateTimeMs = timeMs + 20;
    }
    this._publishLidar();
    if (timeMs >= this.nextCameraTimeMs) {
      this._publishNextCamera();
      const rate = this.demo.sensorSuite.list().cameras[0]?.updateHz ?? 5;
      this.nextCameraTimeMs = timeMs + 1000 / Math.max(1, rate);
    }
  }

  /** Report base, joint and explicit policy status alongside IMU samples. */
  _publishState() {
    const robotState = keyboardController.getRobotState();
    if (!robotState) return;
    const imu = this.demo.sensorSuite.latestImu;
    if (this._send({
      type: 'state',
      environment: this.demo.params.environment,
      simulationTime: Number(this.demo.data.time),
      robot: robotState,
      policy: { ...keyboardController.getPolicyStatus() },
      imu: imu ? {
        timestamp: imu.timestamp,
        gyroscope: finiteArray(imu.gyroscope),
        accelerometer: finiteArray(imu.accelerometer)
      } : null
    })) this.status.stateFrames++;
  }

  /** Forward each new LiDAR scan and point cloud once. */
  _publishLidar() {
    const planar = this.demo.sensorSuite.latestPlanarLidar;
    if (planar && planar.timestamp !== this.lastPlanarTimestamp) {
      const sent = this._send({
        type: 'scan',
        timestamp: planar.timestamp,
        frame: 'mid360_link',
        angleMin: planar.angleMin,
        angleMax: planar.angleMax,
        angleIncrement: planar.angleIncrement,
        rangeMin: planar.rangeMin,
        rangeMax: planar.rangeMax,
        rangesF32: typedArrayToBase64(planar.ranges)
      });
      this.lastPlanarTimestamp = planar.timestamp;
      if (sent) this.status.sensorFrames++;
    }
    const cloud = this.demo.sensorSuite.latestLidar;
    if (cloud && cloud.timestamp !== this.lastCloudTimestamp) {
      const sent = this._send({
        type: 'pointcloud',
        timestamp: cloud.timestamp,
        frame: 'mid360_link',
        pointCount: cloud.pointsLocal.length / 3,
        xyzF32: typedArrayToBase64(cloud.pointsLocal)
      });
      this.lastCloudTimestamp = cloud.timestamp;
      if (sent) this.status.sensorFrames++;
    }
  }

  /** Capture the front RGB-D camera and encode optical-axis depth in metres. */
  _publishNextCamera() {
    if (this.socket?.bufferedAmount >= MAX_BUFFERED_BYTES) return;
    const cameras = this.demo.sensorSuite.list().cameras;
    if (!cameras.length) return;
    const camera = cameras[this.cameraIndex++ % cameras.length];
    try {
      const frame = this.demo.sensorSuite.captureCamera(camera.id, { depth: camera.depth, updateStatus: false });
      if (this._send({
        type: 'camera',
        timestamp: frame.timestamp,
        id: frame.id,
        frame: 'front_camera_optical_frame',
        width: frame.width,
        height: frame.height,
        rgbaU8: typedArrayToBase64(frame.rgba),
        fovyDeg: Number(this.demo.model.cam_fovy[this.demo.sensorSuite.cameras.get(frame.id).cameraId]),
        depth: frame.depth ? {
          width: frame.depth.width,
          height: frame.depth.height,
          dataF32: typedArrayToBase64(frame.depth.data)
        } : null
      })) this.status.sensorFrames++;
    } catch (error) {
      this.status.lastError = `Camera publish failed: ${error}`;
    }
  }
}
