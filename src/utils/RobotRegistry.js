const DEFAULT_INDEX_PATH = './assets/robots/index.json';

function resolvePackagePath(packagePath, relativePath) {
  return relativePath ? `${packagePath}/${relativePath.replace(/^\.\//, '')}` : null;
}

/** Validate and normalize one self-contained robot package manifest. */
export function normalizeRobotPackage(metadata, packagePath, expectedId = null) {
  if (!metadata || metadata.schemaVersion !== 1 || typeof metadata.id !== 'string' ||
      typeof metadata.label !== 'string' || typeof metadata.model !== 'string' ||
      typeof metadata.files !== 'string') {
    throw new Error(`Invalid robot package metadata: ${packagePath}`);
  }
  if (expectedId && metadata.id !== expectedId) {
    throw new Error(`Robot package ID mismatch: expected ${expectedId}, got ${metadata.id}`);
  }
  if (metadata.controller && (typeof metadata.controller.module !== 'string' ||
      typeof metadata.controller.export !== 'string')) {
    throw new Error(`${metadata.id}: controller requires module and export`);
  }
  if (metadata.ikActuators && (!Array.isArray(metadata.ikActuators) ||
      metadata.ikActuators.some((name) => typeof name !== 'string'))) {
    throw new Error(`${metadata.id}: ikActuators must be an array of strings`);
  }
  if (metadata.controlledActuators && (!Array.isArray(metadata.controlledActuators) ||
      metadata.controlledActuators.some((name) => typeof name !== 'string'))) {
    throw new Error(`${metadata.id}: controlledActuators must be an array of strings`);
  }
  if (metadata.dragRootBody && typeof metadata.dragRootBody !== 'string') {
    throw new Error(`${metadata.id}: dragRootBody must be a body name`);
  }
  if (metadata.sensors && (typeof metadata.sensors !== 'object' ||
      (metadata.sensors.lidar && (typeof metadata.sensors.lidar.site !== 'string' ||
        !Number.isInteger(metadata.sensors.lidar.raysPerScan) || metadata.sensors.lidar.raysPerScan <= 0)) ||
      (metadata.sensors.imu && (typeof metadata.sensors.imu.gyroscope !== 'string' ||
        typeof metadata.sensors.imu.accelerometer !== 'string')) ||
      (metadata.sensors.cameras && (!Array.isArray(metadata.sensors.cameras) ||
        metadata.sensors.cameras.some((camera) => typeof camera.id !== 'string' ||
          typeof camera.camera !== 'string' || !Number.isInteger(camera.width) ||
          !Number.isInteger(camera.height)))))) {
    throw new Error(`${metadata.id}: invalid sensors configuration`);
  }

  const sensors = metadata.sensors ? Object.freeze({
    ...metadata.sensors,
    lidar: metadata.sensors.lidar ? Object.freeze({ ...metadata.sensors.lidar }) : null,
    imu: metadata.sensors.imu ? Object.freeze({ ...metadata.sensors.imu }) : null,
    cameras: Object.freeze((metadata.sensors.cameras ?? []).map((camera) => Object.freeze({ ...camera })))
  }) : null;

  return Object.freeze({
    ...metadata,
    packagePath,
    modelPath: resolvePackagePath(packagePath, metadata.model),
    filesPath: resolvePackagePath(packagePath, metadata.files),
    controller: metadata.controller ? Object.freeze({
      ...metadata.controller,
      modulePath: resolvePackagePath(packagePath, metadata.controller.module)
    }) : null,
    sensors,
    ikActuators: Object.freeze([...(metadata.ikActuators ?? [])]),
    controlledActuators: Object.freeze([...(metadata.controlledActuators ?? [])])
  });
}

/** Load the root index and each package-local robot.json exactly once. */
export class RobotRegistry {
  constructor(indexPath = DEFAULT_INDEX_PATH) {
    this.indexPath = indexPath;
    this.defaultRobot = null;
    this.robots = new Map();
    this.initializationPromise = null;
  }

  async initialize() {
    if (this.initializationPromise) return this.initializationPromise;
    this.initializationPromise = this._load();
    return this.initializationPromise;
  }

  /** Fetch package metadata in registry order so the UI order is deterministic. */
  async _load() {
    const response = await fetch(this.indexPath, { cache: 'no-store' });
    if (!response.ok) throw new Error(`Robot index not found: ${this.indexPath}`);
    const index = await response.json();
    if (index.schemaVersion !== 1 || !Array.isArray(index.robots) ||
        typeof index.defaultRobot !== 'string') {
      throw new Error('Unsupported robot index');
    }
    const ids = index.robots.map((entry) => typeof entry === 'string' ? entry : entry.id);
    if (new Set(ids).size !== ids.length) {
      throw new Error('Robot index contains duplicate IDs');
    }

    for (const id of ids.filter((id) => id === 'go2')) {
      if (typeof id !== 'string' || !id || id.includes('/') || id.includes('..')) {
        throw new Error(`Invalid robot ID in index: ${id}`);
      }
      const packagePath = `./assets/robots/${id}`;
      const metadataResponse = await fetch(`${packagePath}/robot.json`, { cache: 'no-store' });
      if (!metadataResponse.ok) throw new Error(`Robot metadata not found: ${id}`);
      this.robots.set(id, normalizeRobotPackage(await metadataResponse.json(), packagePath, id));
    }
    if (!this.robots.has('go2')) {
      throw new Error('Go2 is not registered');
    }
    this.defaultRobot = 'go2';
    return this;
  }

  get(id) {
    return this.robots.get(id) ?? null;
  }

  list() {
    return [...this.robots.keys()];
  }

  getOptions() {
    return Object.fromEntries([...this.robots.values()].map((robot) => [robot.label, robot.id]));
  }
}
