import * as THREE from 'three';

// Generated scene collision geometry lives in group 3 and task objects use
// group 5. Robot collision geometry remains in group 4 and is intentionally
// excluded so simulated sensors do not report any part of the Go2 subtree.
/** Select world collision groups in WASM-owned memory for mj_multiRay. */
function createSensorGeomGroupMask(mujoco) {
  // mujoco-js 0.0.7 interprets TypedArray.byteOffset as a WASM pointer. Store
  // the bytes 00 00 00 01 00 00 in WASM-owned memory and expose that pointer.
  const storage = mujoco.IntBuffer.FromArray([0x01000000, 0x00000100]);
  return {
    storage,
    argument: { byteOffset: storage.GetPointer(), length: 6 }
  };
}
const GOLDEN_ANGLE = Math.PI * (3 - Math.sqrt(5));
const ELEVATION_SEQUENCE = 0.7548776662466927;

function decodeNames(model, count, addresses) {
  const decoder = new TextDecoder('utf-8');
  const result = new Map();
  for (let index = 0; index < count; index++) {
    const address = addresses[index];
    let end = address;
    while (end < model.names.length && model.names[end] !== 0) end++;
    result.set(decoder.decode(model.names.subarray(address, end)), index);
  }
  return result;
}

function multiplyMat3(matrix, offset, x, y, z) {
  return [
    matrix[offset] * x + matrix[offset + 1] * y + matrix[offset + 2] * z,
    matrix[offset + 3] * x + matrix[offset + 4] * y + matrix[offset + 5] * z,
    matrix[offset + 6] * x + matrix[offset + 7] * y + matrix[offset + 8] * z
  ];
}

function swizzlePosition(x, y, z, target) {
  return target.set(x, z, -y);
}

function finite(value, fallback = 0) {
  return Number.isFinite(value) ? value : fallback;
}

/** Generate a deterministic, non-repeating approximation of the MID-360 scan pattern. */
export function generateMid360Directions(count, verticalFovDeg = [-7, 52]) {
  if (!Number.isInteger(count) || count <= 0) throw new Error('LiDAR ray count must be positive');
  const [minimumDeg, maximumDeg] = verticalFovDeg;
  const minimumSin = Math.sin(THREE.MathUtils.degToRad(minimumDeg));
  const maximumSin = Math.sin(THREE.MathUtils.degToRad(maximumDeg));
  const directions = new Float64Array(count * 3);
  for (let index = 0; index < count; index++) {
    const azimuth = index * GOLDEN_ANGLE;
    const elevationUnit = (0.5 + index * ELEVATION_SEQUENCE) % 1;
    const elevation = Math.asin(minimumSin + (maximumSin - minimumSin) * elevationUnit);
    const horizontal = Math.cos(elevation);
    directions[index * 3] = horizontal * Math.cos(azimuth);
    directions[index * 3 + 1] = horizontal * Math.sin(azimuth);
    directions[index * 3 + 2] = Math.sin(elevation);
  }
  return directions;
}

/** Generate MuJoCo camera-frame rays: +X right, +Y up, camera view along -Z. */
export function generatePinholeDirections(width, height, fovyDeg) {
  if (!Number.isInteger(width) || !Number.isInteger(height) || width <= 0 || height <= 0) {
    throw new Error('Camera ray dimensions must be positive integers');
  }
  const tanHalfY = Math.tan(THREE.MathUtils.degToRad(fovyDeg) * 0.5);
  const tanHalfX = tanHalfY * width / height;
  const directions = new Float64Array(width * height * 3);
  let offset = 0;
  for (let row = 0; row < height; row++) {
    const y = (1 - 2 * (row + 0.5) / height) * tanHalfY;
    for (let column = 0; column < width; column++) {
      const x = (2 * (column + 0.5) / width - 1) * tanHalfX;
      const inverseLength = 1 / Math.hypot(x, y, 1);
      directions[offset++] = x * inverseLength;
      directions[offset++] = y * inverseLength;
      directions[offset++] = -inverseLength;
    }
  }
  return directions;
}

/** Generate an ordered counter-clockwise planar scan in the LiDAR frame. */
export function generatePlanarDirections(count, elevationDeg = 0) {
  if (!Number.isInteger(count) || count < 2) throw new Error('Planar LiDAR ray count must be at least 2');
  const elevation = THREE.MathUtils.degToRad(elevationDeg);
  const horizontal = Math.cos(elevation);
  const directions = new Float64Array(count * 3);
  for (let index = 0; index < count; index++) {
    const azimuth = -Math.PI + index * (2 * Math.PI / count);
    directions[index * 3] = horizontal * Math.cos(azimuth);
    directions[index * 3 + 1] = horizontal * Math.sin(azimuth);
    directions[index * 3 + 2] = Math.sin(elevation);
  }
  return directions;
}

function flipRgbaRows(source, width, height) {
  const result = new Uint8Array(source.length);
  const rowBytes = width * 4;
  for (let row = 0; row < height; row++) {
    result.set(source.subarray(row * rowBytes, (row + 1) * rowBytes), (height - row - 1) * rowBytes);
  }
  return result;
}

export class RobotSensorSuite {
  constructor(mujoco, renderer = null, scene = null) {
    this.mujoco = mujoco;
    this.renderer = renderer;
    this.scene = scene;
    this.model = null;
    this.data = null;
    this.config = null;
    this.lidar = null;
    this.imu = null;
    this.cameras = new Map();
    this.latestLidar = null;
    this.latestPlanarLidar = null;
    this.latestImu = null;
    this.showLidar = false;
    this.status = this._newStatus();
    this.liveCameraId = null;
    this.liveFrameCount = 0;
    this.onLiveCameraChange = null;
    this.sensorGeomGroupMask = createSensorGeomGroupMask(mujoco);

    this.pointGeometry = new THREE.BufferGeometry();
    this.pointMaterial = new THREE.PointsMaterial({
      color: 0x35f28b,
      size: 0.025,
      sizeAttenuation: true,
      transparent: true,
      opacity: 0.85
    });
    this.pointCloud = new THREE.Points(this.pointGeometry, this.pointMaterial);
    this.pointCloud.name = 'Robot LiDAR Preview';
    this.pointCloud.frustumCulled = false;
    this.pointCloud.visible = false;
    this.scene?.add(this.pointCloud);
    this.previewElement = null;
    this.previewKey = null;
    this.previewTitle = null;
    this.previewCanvases = null;
  }

  _newStatus() {
    return {
      lidar: 'Unavailable',
      imu: 'Unavailable',
      camera: 'Not captured'
    };
  }

  _releaseModelResources() {
    this.stopLiveCamera();
    if (this.lidar) {
      this.lidar.geomIds.delete();
      this.lidar.distances.delete();
      this.lidar.planarGeomIds.delete();
      this.lidar.planarDistances.delete();
    }
    for (const camera of this.cameras.values()) {
      camera.target?.dispose();
      camera.depthGeomIds?.delete();
      camera.depthDistances?.delete();
    }
    this.lidar = null;
    this.imu = null;
    this.cameras.clear();
    this.latestLidar = null;
    this.latestPlanarLidar = null;
    this.latestImu = null;
    this.pointGeometry.deleteAttribute('position');
    this.pointGeometry.setDrawRange(0, 0);
    this.pointCloud.visible = false;
    this._removeCameraPreview();
  }

  configure(robot, model, data) {
    this._releaseModelResources();
    this.model = model;
    this.data = data;
    this.config = robot?.sensors ?? null;
    Object.assign(this.status, this._newStatus());
    if (!this.config) return false;

    const sites = decodeNames(model, model.nsite, model.name_siteadr);
    const sensors = decodeNames(model, model.nsensor, model.name_sensoradr);
    const cameras = decodeNames(model, model.ncam, model.name_camadr);

    if (this.config.lidar) {
      const config = this.config.lidar;
      const siteId = sites.get(config.site);
      if (siteId === undefined) throw new Error(`LiDAR site not found: ${config.site}`);
      const rayCount = config.raysPerScan;
      const planarRayCount = config.planarRays ?? 720;
      this.lidar = {
        config,
        siteId,
        siteBodyId: model.site_bodyid[siteId],
        localDirections: generateMid360Directions(rayCount, config.verticalFovDeg),
        scanLocalDirections: new Float32Array(rayCount * 3),
        worldDirections: new Array(rayCount * 3).fill(0),
        geomIds: new this.mujoco.IntBuffer(rayCount),
        distances: new this.mujoco.DoubleBuffer(rayCount),
        visualPoints: new Float32Array(rayCount * 3),
        planarLocalDirections: generatePlanarDirections(planarRayCount, config.planarElevationDeg ?? 0),
        planarWorldDirections: new Array(planarRayCount * 3).fill(0),
        planarGeomIds: new this.mujoco.IntBuffer(planarRayCount),
        planarDistances: new this.mujoco.DoubleBuffer(planarRayCount),
        nextScanTimeMs: 0,
        scanIndex: 0
      };
      this.pointGeometry.setAttribute('position', new THREE.BufferAttribute(this.lidar.visualPoints, 3));
      this.status.lidar = `${config.label}: ready`;
    }

    if (this.config.imu) {
      const config = this.config.imu;
      const gyroscopeId = sensors.get(config.gyroscope);
      const accelerometerId = sensors.get(config.accelerometer);
      if (gyroscopeId === undefined) throw new Error(`Gyroscope not found: ${config.gyroscope}`);
      if (accelerometerId === undefined) throw new Error(`Accelerometer not found: ${config.accelerometer}`);
      this.imu = { config, gyroscopeId, accelerometerId };
      this.status.imu = `${config.label}: ready`;
    }

    for (const config of this.config.cameras ?? []) {
      const cameraId = cameras.get(config.camera);
      if (cameraId === undefined) throw new Error(`Camera not found: ${config.camera}`);
      this.cameras.set(config.id, {
        config,
        cameraId,
        cameraBodyId: model.cam_bodyid[cameraId],
        threeCamera: new THREE.PerspectiveCamera(
          model.cam_fovy[cameraId],
          config.width / config.height,
          0.01,
          config.maxDepth ?? 100
        ),
        target: null,
        depthLocalDirections: null,
        depthWorldDirections: null,
        depthGeomIds: null,
        depthDistances: null
      });
    }
    return true;
  }

  update(timeMs) {
    if (!this.config || !this.model || !this.data) return;
    if (this.imu) this.readImu();
    if (this.lidar && timeMs >= this.lidar.nextScanTimeMs) {
      this.scanLidar();
      this.scanPlanarLidar();
      this.lidar.nextScanTimeMs = timeMs + 1000 / this.lidar.config.updateHz;
    }
  }

  readImu() {
    if (!this.imu) return null;
    const readVector = (sensorId) => {
      const address = this.model.sensor_adr[sensorId];
      return Array.from(this.data.sensordata.subarray(address, address + 3), Number);
    };
    this.latestImu = {
      timestamp: Number(this.data.time),
      gyroscope: readVector(this.imu.gyroscopeId),
      accelerometer: readVector(this.imu.accelerometerId)
    };
    const gyroNorm = Math.hypot(...this.latestImu.gyroscope);
    const accelNorm = Math.hypot(...this.latestImu.accelerometer);
    this.status.imu = `gyro ${gyroNorm.toFixed(3)} rad/s | accel ${accelNorm.toFixed(2)} m/s2`;
    return this.latestImu;
  }

  scanLidar() {
    if (!this.lidar) return null;
    const { config, siteId, localDirections, scanLocalDirections, worldDirections } = this.lidar;
    const originOffset = siteId * 3;
    const matrixOffset = siteId * 9;
    const origin = [
      this.data.site_xpos[originOffset],
      this.data.site_xpos[originOffset + 1],
      this.data.site_xpos[originOffset + 2]
    ];
    const phase = this.lidar.scanIndex * GOLDEN_ANGLE;
    const cosine = Math.cos(phase);
    const sine = Math.sin(phase);
    for (let index = 0; index < config.raysPerScan; index++) {
      const offset = index * 3;
      const localX = cosine * localDirections[offset] - sine * localDirections[offset + 1];
      const localY = sine * localDirections[offset] + cosine * localDirections[offset + 1];
      scanLocalDirections[offset] = localX;
      scanLocalDirections[offset + 1] = localY;
      scanLocalDirections[offset + 2] = localDirections[offset + 2];
      const [x, y, z] = multiplyMat3(
        this.data.site_xmat,
        matrixOffset,
        localX,
        localY,
        localDirections[offset + 2]
      );
      worldDirections[offset] = x;
      worldDirections[offset + 1] = y;
      worldDirections[offset + 2] = z;
    }

    this.mujoco.mj_multiRay(
      this.model,
      this.data,
      origin,
      worldDirections,
      this.sensorGeomGroupMask.argument,
      1,
      this.lidar.siteBodyId,
      this.lidar.geomIds,
      this.lidar.distances,
      config.raysPerScan,
      config.maxRange
    );

    const rawDistances = this.lidar.distances.GetView();
    const rawGeomIds = this.lidar.geomIds.GetView();
    const ranges = new Float32Array(config.raysPerScan);
    const pointsMujoco = new Float32Array(config.raysPerScan * 3);
    const pointsLocal = new Float32Array(config.raysPerScan * 3);
    const hitGeomIds = new Int32Array(config.raysPerScan);
    let validPoints = 0;
    let minimumRange = Infinity;
    let maximumRange = 0;
    for (let index = 0; index < config.raysPerScan; index++) {
      const distance = rawDistances[index];
      if (distance < config.minRange || distance > config.maxRange) {
        ranges[index] = Infinity;
        continue;
      }
      ranges[index] = distance;
      minimumRange = Math.min(minimumRange, distance);
      maximumRange = Math.max(maximumRange, distance);
      const directionOffset = index * 3;
      const pointOffset = validPoints * 3;
      const x = origin[0] + distance * worldDirections[directionOffset];
      const y = origin[1] + distance * worldDirections[directionOffset + 1];
      const z = origin[2] + distance * worldDirections[directionOffset + 2];
      pointsMujoco[pointOffset] = x;
      pointsMujoco[pointOffset + 1] = y;
      pointsMujoco[pointOffset + 2] = z;
      pointsLocal[pointOffset] = distance * scanLocalDirections[directionOffset];
      pointsLocal[pointOffset + 1] = distance * scanLocalDirections[directionOffset + 1];
      pointsLocal[pointOffset + 2] = distance * scanLocalDirections[directionOffset + 2];
      this.lidar.visualPoints[pointOffset] = x;
      this.lidar.visualPoints[pointOffset + 1] = z;
      this.lidar.visualPoints[pointOffset + 2] = -y;
      hitGeomIds[validPoints] = rawGeomIds[index];
      validPoints++;
    }
    this.lidar.scanIndex++;
    this.pointGeometry.setDrawRange(0, validPoints);
    this.pointGeometry.attributes.position.needsUpdate = true;
    this.pointCloud.visible = this.showLidar && validPoints > 0;
    this.latestLidar = {
      timestamp: Number(this.data.time),
      frame: config.site,
      origin: Float32Array.from(origin),
      ranges,
      points: pointsMujoco.slice(0, validPoints * 3),
      pointsLocal: pointsLocal.slice(0, validPoints * 3),
      hitGeomIds: hitGeomIds.slice(0, validPoints),
      validPoints,
      minimumRange,
      maximumRange
    };
    const rangeText = validPoints ? `${minimumRange.toFixed(2)}-${maximumRange.toFixed(2)} m` : 'no returns';
    this.status.lidar = `${config.label}: ${validPoints}/${config.raysPerScan}, ${rangeText}`;
    return this.latestLidar;
  }

  scanPlanarLidar() {
    if (!this.lidar) return null;
    const {
      config, siteId, siteBodyId, planarLocalDirections, planarWorldDirections,
      planarGeomIds, planarDistances
    } = this.lidar;
    const originOffset = siteId * 3;
    const matrixOffset = siteId * 9;
    const origin = Array.from(this.data.site_xpos.subarray(originOffset, originOffset + 3));
    const rayCount = planarLocalDirections.length / 3;
    for (let index = 0; index < rayCount; index++) {
      const offset = index * 3;
      const [x, y, z] = multiplyMat3(
        this.data.site_xmat,
        matrixOffset,
        planarLocalDirections[offset],
        planarLocalDirections[offset + 1],
        planarLocalDirections[offset + 2]
      );
      planarWorldDirections[offset] = x;
      planarWorldDirections[offset + 1] = y;
      planarWorldDirections[offset + 2] = z;
    }
    this.mujoco.mj_multiRay(
      this.model,
      this.data,
      origin,
      planarWorldDirections,
      this.sensorGeomGroupMask.argument,
      1,
      siteBodyId,
      planarGeomIds,
      planarDistances,
      rayCount,
      config.maxRange
    );
    const rawDistances = planarDistances.GetView();
    const rawGeomIds = planarGeomIds.GetView();
    const ranges = new Float32Array(rayCount);
    const hitGeomIds = new Int32Array(rayCount);
    let validPoints = 0;
    for (let index = 0; index < rayCount; index++) {
      const distance = rawDistances[index];
      ranges[index] = distance >= config.minRange && distance <= config.maxRange ? distance : Infinity;
      hitGeomIds[index] = rawGeomIds[index];
      if (Number.isFinite(ranges[index])) validPoints++;
    }
    this.latestPlanarLidar = {
      timestamp: Number(this.data.time),
      frame: config.site,
      angleMin: -Math.PI,
      angleMax: Math.PI - 2 * Math.PI / rayCount,
      angleIncrement: 2 * Math.PI / rayCount,
      rangeMin: config.minRange,
      rangeMax: config.maxRange,
      ranges,
      hitGeomIds,
      validPoints
    };
    return this.latestPlanarLidar;
  }

  setLidarVisible(visible) {
    this.showLidar = Boolean(visible);
    this.pointCloud.visible = this.showLidar && Boolean(this.latestLidar?.validPoints);
  }

  _syncThreeCamera(camera, aspect = camera.config.width / camera.config.height) {
    const offset = camera.cameraId * 3;
    const matrixOffset = camera.cameraId * 9;
    const position = swizzlePosition(
      this.data.cam_xpos[offset],
      this.data.cam_xpos[offset + 1],
      this.data.cam_xpos[offset + 2],
      camera.threeCamera.position
    );
    const forwardMujoco = multiplyMat3(this.data.cam_xmat, matrixOffset, 0, 0, -1);
    const upMujoco = multiplyMat3(this.data.cam_xmat, matrixOffset, 0, 1, 0);
    const forward = swizzlePosition(...forwardMujoco, new THREE.Vector3());
    const up = swizzlePosition(...upMujoco, new THREE.Vector3());
    camera.threeCamera.up.copy(up).normalize();
    camera.threeCamera.lookAt(position.clone().add(forward));
    if (camera.threeCamera.aspect !== aspect) {
      camera.threeCamera.aspect = aspect;
      camera.threeCamera.updateProjectionMatrix();
    }
    camera.threeCamera.updateMatrixWorld(true);
  }

  _captureRgb(camera) {
    if (!this.renderer || !this.scene) throw new Error('RGB capture requires a WebGL renderer and scene');
    const { width, height } = camera.config;
    if (!camera.target) {
      camera.target = new THREE.WebGLRenderTarget(width, height, {
        format: THREE.RGBAFormat,
        type: THREE.UnsignedByteType,
        depthBuffer: true
      });
    }
    this._syncThreeCamera(camera);
    const previousTarget = this.renderer.getRenderTarget();
    const previousClearColor = this.renderer.getClearColor(new THREE.Color());
    const previousClearAlpha = this.renderer.getClearAlpha();
    const lidarWasVisible = this.pointCloud.visible;
    this.pointCloud.visible = false;
    const pixels = new Uint8Array(width * height * 4);
    const gl = this.renderer.getContext();
    const previousPixelPackBuffer = gl.getParameter(gl.PIXEL_PACK_BUFFER_BINDING);
    try {
      this.renderer.setRenderTarget(camera.target);
      this.renderer.setClearColor(0x111820, 1);
      this.renderer.clear(true, true, true);
      this.renderer.render(this.scene, camera.threeCamera);
      // Spark uses asynchronous PBO readback. Three's synchronous helper
      // requires the default pack target, then Spark needs its binding back.
      gl.bindBuffer(gl.PIXEL_PACK_BUFFER, null);
      this.renderer.readRenderTargetPixels(camera.target, 0, 0, width, height, pixels);
    } finally {
      gl.bindBuffer(gl.PIXEL_PACK_BUFFER, previousPixelPackBuffer);
      this.renderer.setRenderTarget(previousTarget);
      this.renderer.setClearColor(previousClearColor, previousClearAlpha);
      this.pointCloud.visible = lidarWasVisible;
    }
    return flipRgbaRows(pixels, width, height);
  }

  /** Return optical-axis Z depth, using longer corner rays and excluding robot collision groups. */
  _captureDepth(camera) {
    const config = camera.config;
    if (!config.depth) return null;
    const width = config.depthWidth ?? config.width;
    const height = config.depthHeight ?? config.height;
    const rayCount = width * height;
    if (!camera.depthLocalDirections) {
      camera.depthLocalDirections = generatePinholeDirections(
        width,
        height,
        this.model.cam_fovy[camera.cameraId]
      );
      camera.depthWorldDirections = new Array(rayCount * 3).fill(0);
      camera.depthGeomIds = new this.mujoco.IntBuffer(rayCount);
      camera.depthDistances = new this.mujoco.DoubleBuffer(rayCount);
    }
    const matrixOffset = camera.cameraId * 9;
    for (let index = 0; index < rayCount; index++) {
      const offset = index * 3;
      const [x, y, z] = multiplyMat3(
        this.data.cam_xmat,
        matrixOffset,
        camera.depthLocalDirections[offset],
        camera.depthLocalDirections[offset + 1],
        camera.depthLocalDirections[offset + 2]
      );
      camera.depthWorldDirections[offset] = x;
      camera.depthWorldDirections[offset + 1] = y;
      camera.depthWorldDirections[offset + 2] = z;
    }
    const originOffset = camera.cameraId * 3;
    this.mujoco.mj_multiRay(
      this.model,
      this.data,
      Array.from(this.data.cam_xpos.subarray(originOffset, originOffset + 3)),
      camera.depthWorldDirections,
      this.sensorGeomGroupMask.argument,
      1,
      camera.cameraBodyId,
      camera.depthGeomIds,
      camera.depthDistances,
      rayCount,
      config.maxDepth / -camera.depthLocalDirections[2]
    );
    const raw = camera.depthDistances.GetView();
    const rawGeomIds = camera.depthGeomIds.GetView();
    const depth = new Float32Array(rayCount);
    const geomIds = new Int32Array(rayCount);
    for (let index = 0; index < rayCount; index++) {
      const value = rayRangeToOpticalDepth(raw[index], camera.depthLocalDirections[index * 3 + 2]);
      depth[index] = value >= config.minDepth && value <= config.maxDepth ? value : Infinity;
      geomIds[index] = rawGeomIds[index];
    }
    return { width, height, data: depth, geomIds };
  }

  captureCamera(cameraId, { depth = true, updateStatus = true } = {}) {
    const camera = this.cameras.get(cameraId);
    if (!camera) throw new Error(`Unknown robot camera: ${cameraId}`);
    const frame = {
      id: camera.config.id,
      label: camera.config.label,
      camera: camera.config.camera,
      timestamp: Number(this.data.time),
      width: camera.config.width,
      height: camera.config.height,
      rgba: this._captureRgb(camera),
      depth: depth ? this._captureDepth(camera) : null
    };
    if (updateStatus) {
      this.status.camera = `${frame.label}: ${frame.width}x${frame.height}${frame.depth ? ' + depth' : ''}`;
    }
    return frame;
  }

  startLiveCamera(cameraId) {
    if (!this.cameras.has(cameraId)) throw new Error(`Unknown robot camera: ${cameraId}`);
    this.liveCameraId = cameraId;
    this.liveFrameCount = 0;
    this._removeCameraPreview();
    this.onLiveCameraChange?.(true, cameraId);
  }

  stopLiveCamera() {
    const wasActive = Boolean(this.liveCameraId);
    this.liveCameraId = null;
    if (wasActive) this.onLiveCameraChange?.(false, null);
  }

  getRenderCamera(fallbackCamera) {
    const camera = this.cameras.get(this.liveCameraId);
    if (!camera) return fallbackCamera;
    const canvas = this.renderer?.domElement;
    const aspect = canvas?.clientHeight > 0
      ? canvas.clientWidth / canvas.clientHeight
      : camera.config.width / camera.config.height;
    this._syncThreeCamera(camera, aspect);
    this.liveFrameCount++;
    this.status.camera = `${camera.config.label}: live RGBA view`;
    return camera.threeCamera;
  }

  getLiveCameraState() {
    return {
      active: Boolean(this.liveCameraId),
      cameraId: this.liveCameraId,
      frameCount: this.liveFrameCount
    };
  }

  _removeCameraPreview() {
    this.previewElement?.remove();
    this.previewElement = null;
    this.previewKey = null;
    this.previewTitle = null;
    this.previewCanvases = null;
  }

  _createCameraPreview(frame) {
    this._removeCameraPreview();
    const panel = document.createElement('section');
    panel.id = 'robot-sensor-preview';
    panel.style.cssText = [
      'position:fixed', 'left:16px', 'bottom:76px', 'z-index:1200',
      'width:min(680px,calc(100vw - 32px))', 'max-height:calc(100vh - 110px)',
      'overflow:auto', 'background:#111820', 'color:#f4f7f8', 'border:1px solid #46515b',
      'border-radius:6px', 'padding:12px', 'box-sizing:border-box',
      'font:13px system-ui,sans-serif', 'box-shadow:0 12px 30px rgba(0,0,0,.4)'
    ].join(';');
    const header = document.createElement('div');
    header.style.cssText = 'display:flex;align-items:center;justify-content:space-between;margin-bottom:10px';
    const title = document.createElement('strong');
    const close = document.createElement('button');
    close.type = 'button';
    close.textContent = '×';
    close.title = 'Close sensor preview';
    close.setAttribute('aria-label', 'Close sensor preview');
    close.style.cssText = 'width:30px;height:30px;border:0;background:#29343d;color:white;font-size:22px;cursor:pointer;border-radius:4px';
    close.onclick = () => {
      this.stopLiveCamera();
      this._removeCameraPreview();
    };
    header.append(title, close);
    panel.appendChild(header);

    const images = document.createElement('div');
    images.style.cssText = `display:grid;grid-template-columns:repeat(${frame.depth ? 2 : 1},minmax(0,1fr));gap:10px`;
    const addCanvas = (labelText, width, height) => {
      const region = document.createElement('div');
      const label = document.createElement('div');
      label.textContent = labelText;
      label.style.cssText = 'margin-bottom:5px;color:#b9c6cf';
      const canvas = document.createElement('canvas');
      canvas.width = width;
      canvas.height = height;
      canvas.style.cssText = 'display:block;width:100%;height:auto;background:#000;aspect-ratio:' + width + '/' + height;
      region.append(label, canvas);
      images.appendChild(region);
      return canvas;
    };
    const rgbCanvas = addCanvas('RGBA', frame.width, frame.height);
    let depthCanvas = null;
    if (frame.depth) {
      depthCanvas = addCanvas('Depth', frame.depth.width, frame.depth.height);
    }
    panel.appendChild(images);
    document.body.appendChild(panel);
    this.previewElement = panel;
    this.previewKey = `${frame.id}:${Boolean(frame.depth)}`;
    this.previewTitle = title;
    this.previewCanvases = { rgb: rgbCanvas, depth: depthCanvas };
  }

  showCameraPreview(frame, { live = false } = {}) {
    const key = `${frame.id}:${Boolean(frame.depth)}`;
    if (!this.previewElement?.isConnected || this.previewKey !== key) {
      this._createCameraPreview(frame);
    }
    this.previewTitle.textContent = live ? `${frame.label} - Live RGBA` : frame.label;
    this.previewCanvases.rgb.getContext('2d').putImageData(
      new ImageData(new Uint8ClampedArray(frame.rgba), frame.width, frame.height),
      0,
      0
    );
    if (frame.depth && this.previewCanvases.depth) {
      const grayscale = new Uint8Array(frame.depth.width * frame.depth.height * 4);
      const validDepths = Array.from(frame.depth.data).filter(Number.isFinite).sort((a, b) => a - b);
      const quantile = (fraction, fallback) => validDepths.length
        ? validDepths[Math.floor((validDepths.length - 1) * fraction)]
        : fallback;
      const configuredMaximum = finite(this.cameras.get(frame.id)?.config.maxDepth, 20);
      const displayNear = quantile(0.02, 0);
      const displayFar = quantile(0.98, configuredMaximum);
      const displaySpan = Math.max(displayFar - displayNear, 0.1);
      for (let index = 0; index < frame.depth.data.length; index++) {
        const value = frame.depth.data[index];
        const normalized = Math.min(1, Math.max(0, (value - displayNear) / displaySpan));
        const shade = Number.isFinite(value) ? Math.round(32 + 223 * (1 - normalized)) : 0;
        const offset = index * 4;
        grayscale[offset] = shade;
        grayscale[offset + 1] = shade;
        grayscale[offset + 2] = shade;
        grayscale[offset + 3] = 255;
      }
      this.previewCanvases.depth.getContext('2d').putImageData(
        new ImageData(new Uint8ClampedArray(grayscale), frame.depth.width, frame.depth.height),
        0,
        0
      );
      this.previewCanvases.depth.parentElement.firstElementChild.textContent = validDepths.length
        ? `Depth (${displayNear.toFixed(2)}-${displayFar.toFixed(2)} m)`
        : 'Depth (no returns)';
    }
  }

  list() {
    if (!this.config) return { lidar: null, imu: null, cameras: [] };
    return {
      lidar: this.config.lidar ?? null,
      imu: this.config.imu ?? null,
      cameras: (this.config.cameras ?? []).map((camera) => ({ ...camera }))
    };
  }

  dispose() {
    this._releaseModelResources();
    this.sensorGeomGroupMask.storage.delete();
    this.scene?.remove(this.pointCloud);
    this.pointGeometry.dispose();
    this.pointMaterial.dispose();
  }
}

/** Convert a normalized MuJoCo camera ray's range into positive optical-axis depth. */
export function rayRangeToOpticalDepth(range, directionZ) {
  return range >= 0 && directionZ < 0 ? range * -directionZ : Infinity;
}
