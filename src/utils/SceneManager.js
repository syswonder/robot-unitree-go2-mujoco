/**
 * Scene Manager
 *
 * Manages loading of modular scenes using MuJoCo's include mechanism.
 * Scene files use <include file="robot.xml"/> to include robot definitions.
 */

import { RobotLoader } from './RobotLoader.js';
import { RobotRegistry } from './RobotRegistry.js';

export class SceneManager {
  /**
   * Environment configurations
   * All environments are in assets/environments/{envName}/
   */
  static ENV_CONFIGS = {
    'basic': {
      xmlPath: './assets/environments/basic.xml',
      spzPath: null,
      description: 'Basic environment with light and floor only'
    }
  };

  constructor(mujoco) {
    this.mujoco = mujoco;
    this.robotRegistry = new RobotRegistry();
    this.robotLoader = new RobotLoader(mujoco, this.robotRegistry);
    this.currentEnv = null;
    this.currentRobot = null;
    this.scenePath = null;
    this.customSpzData = null;  // Store custom SPZ data from user upload
    this.customCollisionXml = null;  // Store custom collision XML content
    this.defaultEnvironment = 'basic';
    this.environmentManifestPromise = null;
    this.transformCache = new Map();
  }

  /** Load the root robot index and every package-local robot.json. */
  async initializeRobots() {
    await this.robotRegistry.initialize();
    return this.robotRegistry;
  }

  /** Load generated environment paths before the first MuJoCo scene is created. */
  async initializeEnvironments(manifestPath = './assets/environments/manifest.json') {
    if (this.environmentManifestPromise) return this.environmentManifestPromise;
    this.environmentManifestPromise = (async () => {
      const response = await fetch(manifestPath);
      if (!response.ok) {
        throw new Error(`Environment manifest not found: ${manifestPath}`);
      }
      const manifest = await response.json();
      if (manifest.schemaVersion !== 2 || !Array.isArray(manifest.environments)) {
        throw new Error('Unsupported environment manifest');
      }

      for (const environment of manifest.environments) {
        if (!environment.id || !environment.xmlPath || !['spz', 'mesh'].includes(environment.visualMode)) {
          throw new Error('Environment manifest contains an incomplete entry');
        }
        if (environment.visualMode === 'spz' && (!environment.spzPath || !environment.transformPath)) {
          throw new Error(`${environment.id}: SPZ environment requires spzPath and transformPath`);
        }
        if (environment.visualMode === 'mesh' && !environment.filesPath) {
          throw new Error(`${environment.id}: mesh environment requires filesPath`);
        }
        let spawn = environment.spawn ?? null;
        if (environment.spawnPath) {
          const spawnResponse = await fetch(environment.spawnPath, { cache: 'no-store' });
          if (!spawnResponse.ok) throw new Error(`${environment.id}: spawn file not found`);
          spawn = await spawnResponse.json();
        }
        SceneManager.ENV_CONFIGS[environment.id] = {
          xmlPath: environment.xmlPath,
          visualMode: environment.visualMode,
          spzPath: environment.spzPath ?? null,
          transformPath: environment.transformPath ?? null,
          filesPath: environment.filesPath ?? null,
          objectsPath: environment.objectsPath ?? null,
          spawn,
          camera: environment.camera ?? null,
          label: environment.label ?? environment.id,
          description: `${environment.label ?? environment.id} ${environment.visualMode.toUpperCase()} environment`
        };
      }
      if (!SceneManager.ENV_CONFIGS[manifest.defaultEnvironment]) {
        throw new Error(`Invalid default environment: ${manifest.defaultEnvironment}`);
      }
      this.defaultEnvironment = manifest.defaultEnvironment;
      return manifest;
    })();
    return this.environmentManifestPromise;
  }

  getDefaultEnvironment() {
    return this.defaultEnvironment;
  }

  /** Return the current collision-aware robot spawn without exposing mutable manifest state. */
  getCurrentSpawnPosition() {
    const position = SceneManager.ENV_CONFIGS[this.currentEnv]?.spawn?.position;
    return Array.isArray(position) ? [...position] : null;
  }

  getCurrentCameraPreset() {
    const camera = SceneManager.ENV_CONFIGS[this.currentEnv]?.camera;
    if (!camera) return null;
    return {
      position: [...camera.position],
      target: [...camera.target]
    };
  }

  /**
   * Load a scene using MuJoCo's include mechanism.
   * Creates a scene XML that uses <include> to bring in robot and objects.
   *
   * @param {string} envName - Environment identifier from the generated manifest
   * @param {string} robotName - Robot name (e.g., 'xlerobot', 'panda')
   * @returns {Promise<string>} - Path to the scene XML in virtual filesystem
   */
  async loadModularScene(envName, robotName) {
    console.log(`Loading modular scene: env=${envName}, robot=${robotName}`);

    try {
      if (!this.robotRegistry.get(robotName)) {
        throw new Error(`Unknown robot: ${robotName}`);
      }

      const envConfig = SceneManager.ENV_CONFIGS[envName];
      if (!envConfig) {
        throw new Error(`Unknown environment: ${envName}`);
      }

      const vfsSceneDir = `/working/scenes/${robotName}`;
      this._ensureDir('/working/scenes');
      this._resetDirectory(vfsSceneDir);

      // Mesh environments carry their own static visual and collision assets.
      const hasObjects = await this._copyEnvironmentToDir(envConfig, vfsSceneDir);

      // Copy robot files to scene directory
      await this._copyRobotToDir(robotName, vfsSceneDir, envConfig.spawn);

      // Load environment XML and create scene
      const envResponse = await fetch(envConfig.xmlPath);
      if (!envResponse.ok) {
        throw new Error(`Environment XML not found: ${envConfig.xmlPath}`);
      }
      const envXml = await envResponse.text();

      const sceneName = `${envName}_${robotName}`;
      const sceneXml = this._createSceneXml(envXml, robotName, hasObjects, sceneName);
      this._writeToFS(`${vfsSceneDir}/scene.xml`, sceneXml);

      this.currentEnv = envName;
      this.currentRobot = robotName;
      this.scenePath = `scenes/${robotName}/scene.xml`;

      console.log(`Scene loaded: ${this.scenePath}`);
      return this.scenePath;

    } catch (error) {
      console.error('Scene loading failed:', error);
      throw error;
    }
  }

  /**
   * Create scene XML by inserting include statements into environment XML
   */
  _createSceneXml(envXml, robotName, hasObjects, sceneName) {
    // Parse environment XML
    const parser = new DOMParser();
    const doc = parser.parseFromString(envXml, 'text/xml');

    if (doc.querySelector('parsererror')) {
      throw new Error('Failed to parse environment XML');
    }

    // Update model name
    const mujoco = doc.documentElement;
    mujoco.setAttribute('model', sceneName);

    // Create include element for robot
    const robotInclude = doc.createElement('include');
    robotInclude.setAttribute('file', `${robotName}.xml`);

    // Insert include at the beginning (after mujoco tag)
    mujoco.insertBefore(robotInclude, mujoco.firstChild);

    // Add objects include if exists
    if (hasObjects) {
      const objectsInclude = doc.createElement('include');
      objectsInclude.setAttribute('file', 'objects.xml');
      // Insert after robot include
      mujoco.insertBefore(objectsInclude, robotInclude.nextSibling);
    }

    // Serialize back to string
    const serializer = new XMLSerializer();
    let result = serializer.serializeToString(doc);

    // Clean up XML
    result = result.replace(/(<\?xml[^?]*\?>)/g, '');
    result = result.replace(/\s+xmlns="[^"]*"/g, '');
    result = result.replace(/\s+xmlns:[a-z]+="[^"]*"/g, '');
    result = '<?xml version="1.0" encoding="UTF-8"?>\n' + result.trim();

    return result;
  }

  /**
   * Write content to MuJoCo virtual filesystem
   */
  _writeToFS(path, content) {
    // Try to remove existing file
    try {
      this.mujoco.FS.unlink(path);
    } catch (e) {
      // File doesn't exist, that's fine
    }

    // Write new file
    if (typeof content === 'string') {
      this.mujoco.FS.writeFile(path, content);
    } else {
      this.mujoco.FS.writeFile(path, content);
    }
    console.log(`Written to VFS: ${path}`);
  }

  /**
   * Ensure directory exists in virtual filesystem
   */
  _ensureDir(path) {
    try {
      if (!this.mujoco.FS.analyzePath(path).exists) {
        this.mujoco.FS.mkdir(path);
      }
    } catch (e) {
      // Directory might already exist
    }
  }

  /** Remove the previous modular package so large mesh assets do not accumulate in MEMFS. */
  _resetDirectory(path) {
    const fs = this.mujoco.FS;
    const removeContents = (directory) => {
      for (const name of fs.readdir(directory)) {
        if (name === '.' || name === '..') continue;
        const child = `${directory}/${name}`;
        if (fs.isDir(fs.stat(child).mode)) {
          removeContents(child);
          fs.rmdir(child);
        } else {
          fs.unlink(child);
        }
      }
    };

    if (fs.analyzePath(path).exists) {
      removeContents(path);
    } else {
      this._ensureDir(path);
    }
  }

  /** Copy an environment package into its own namespace in MuJoCo's VFS. */
  async _copyEnvironmentToDir(envConfig, targetDir) {
    if (envConfig.filesPath) {
      const response = await fetch(envConfig.filesPath, { cache: 'no-store' });
      if (!response.ok) throw new Error(`Environment file index not found: ${envConfig.filesPath}`);
      const files = await response.json();
      if (!Array.isArray(files) || !files.length) throw new Error('Environment file index is empty');
      const baseUrl = envConfig.filesPath.slice(0, envConfig.filesPath.lastIndexOf('/'));
      const root = targetDir;

      const normalizedFiles = files.map((rawFile) => {
        const file = String(rawFile).replace(/\\/g, '/').replace(/^\.\//, '');
        if (!file || file.startsWith('/') || file.split('/').includes('..')) {
          throw new Error(`Invalid environment package path: ${rawFile}`);
        }
        return file;
      });

      for (const file of normalizedFiles) {
        let parent = root;
        for (const part of file.split('/').slice(0, -1)) {
          parent += `/${part}`;
          this._ensureDir(parent);
        }
      }

      let nextFile = 0;
      const copyNext = async () => {
        while (nextFile < normalizedFiles.length) {
          const file = normalizedFiles[nextFile++];
          const fileResponse = await fetch(`${baseUrl}/${file}`);
          if (!fileResponse.ok) throw new Error(`Failed to fetch environment asset: ${file}`);
          const extension = file.split('.').pop().toLowerCase();
          const data = ['xml', 'mjcf', 'json', 'txt'].includes(extension)
            ? await fileResponse.text()
            : new Uint8Array(await fileResponse.arrayBuffer());
          this._writeToFS(`${root}/${file}`, data);
        }
      };
      const concurrency = Math.min(8, normalizedFiles.length);
      await Promise.all(Array.from({ length: concurrency }, copyNext));
    }

    if (!envConfig.objectsPath) return false;
    const objectsResponse = await fetch(envConfig.objectsPath, { cache: 'no-store' });
    if (!objectsResponse.ok) throw new Error(`Environment objects file not found: ${envConfig.objectsPath}`);
    this._writeToFS(`${targetDir}/objects.xml`, await objectsResponse.text());
    return true;
  }

  /**
   * Load user-uploaded robot files
   * @param {FileList} files - Files from input[webkitdirectory]
   * @param {string} envName - Environment to load robot into
   * @returns {Promise<string>} - Path to scene XML
   */
  async loadUploadedRobot(files, envName = null) {
    envName = envName ?? this.defaultEnvironment;
    // Parse uploaded files
    const { robotXml, meshFiles, robotName } =
      await this.robotLoader.loadUploadedRobot(files);

    const vfsSceneDir = `/working/scenes/uploaded_${robotName}`;

    const effectiveEnv = (envName === 'custom_spz') ? 'basic' : envName;
    const envConfig = SceneManager.ENV_CONFIGS[effectiveEnv];
    if (!envConfig) throw new Error(`Unknown environment: ${envName}`);

    // Build the scene directory from the selected environment and uploaded robot.
    this._ensureDir('/working/scenes');
    this._resetDirectory(vfsSceneDir);
    const hasObjects = await this._copyEnvironmentToDir(envConfig, vfsSceneDir);
    this._ensureDir(`${vfsSceneDir}/assets`);

    // Write asset files
    for (const [name, buffer] of meshFiles) {
      const meshPath = `${vfsSceneDir}/assets/${name}`;
      this.mujoco.FS.writeFile(meshPath, new Uint8Array(buffer));
    }

    // Fix meshdir and write robot XML
    let fixedRobotXml = robotXml.replace(
      /meshdir="[^"]*"/g,
      `meshdir="./assets/"`
    );
    this._writeToFS(`${vfsSceneDir}/robot.xml`, fixedRobotXml);

    // Load environment XML
    const envResponse = await fetch(envConfig.xmlPath);
    const envXml = await envResponse.text();

    // Create scene XML with include
    const sceneName = `${effectiveEnv}_uploaded_${robotName}`;
    const sceneXml = this._createSceneXml(envXml, 'robot', hasObjects, sceneName);

    // Write scene XML
    this._writeToFS(`${vfsSceneDir}/scene.xml`, sceneXml);

    // Keep custom_spz as currentEnv if that was the original environment
    this.currentEnv = envName;
    this.currentRobot = `uploaded_${robotName}`;
    this.scenePath = `scenes/uploaded_${robotName}/scene.xml`;

    return this.scenePath;
  }

  /**
   * Get current scene info
   */
  getCurrentSceneInfo() {
    return {
      environment: this.currentEnv,
      robot: this.currentRobot,
      scenePath: this.scenePath
    };
  }

  /**
   * Get 3DGS path for current environment
   * @returns {string|null}
   */
  getSpzPath() {
    if (!this.currentEnv) return null;
    const envConfig = SceneManager.ENV_CONFIGS[this.currentEnv];
    return envConfig ? envConfig.spzPath : null;
  }

  getVisualMode(environment = this.currentEnv) {
    return SceneManager.ENV_CONFIGS[environment]?.visualMode ?? null;
  }

  /** Fetch and cache the offline transform for the current environment. */
  async getSpzTransform() {
    if (!this.currentEnv) return null;
    const sceneId = this.currentEnv;
    const envConfig = SceneManager.ENV_CONFIGS[sceneId];
    if (!envConfig?.transformPath) return null;
    if (!this.transformCache.has(envConfig.transformPath)) {
      const pending = fetch(envConfig.transformPath).then(async (response) => {
        if (!response.ok) {
          throw new Error(`Scene transform not found: ${envConfig.transformPath}`);
        }
        const metadata = await response.json();
        if (metadata.schemaVersion !== 1 || metadata.sceneId !== sceneId || !metadata.transform) {
          throw new Error(`Invalid scene transform: ${envConfig.transformPath}`);
        }
        return metadata;
      });
      this.transformCache.set(envConfig.transformPath, pending);
    }
    return this.transformCache.get(envConfig.transformPath);
  }

  /**
   * Get custom SPZ data if available
   * @returns {ArrayBuffer|null}
   */
  getCustomSpzData() {
    return this.customSpzData;
  }

  /**
   * Check if using custom SPZ
   * @returns {boolean}
   */
  hasCustomSpz() {
    return this.customSpzData !== null;
  }

  /**
   * Clear custom SPZ data
   */
  clearCustomSpz() {
    this.customSpzData = null;
  }

  /**
   * Set custom collision XML content
   * @param {string} xmlContent - The collision XML content
   */
  setCustomCollision(xmlContent) {
    this.customCollisionXml = xmlContent;
    console.log('Custom collision XML set');
  }

  /**
   * Clear custom collision XML
   */
  clearCustomCollision() {
    this.customCollisionXml = null;
    console.log('Custom collision XML cleared');
  }

  /**
   * Check if custom collision XML is available
   * @returns {boolean}
   */
  hasCustomCollision() {
    return this.customCollisionXml !== null;
  }

  /**
   * Get custom collision XML content
   * @returns {string|null}
   */
  getCustomCollisionXml() {
    return this.customCollisionXml;
  }

  /**
   * Load a custom SPZ file with basic environment
   * @param {File} spzFile - The SPZ file to load
   * @param {string} robotName - Robot to use (default: none)
   * @returns {Promise<string>} - Path to scene XML
   */
  async loadCustomSpz(spzFile, robotName = null) {
    console.log(`Loading custom SPZ: ${spzFile.name}`);
    this.customSpzData = await spzFile.arrayBuffer();
    return this._setupCustomSpzScene(robotName);
  }

  /**
   * Reload custom SPZ scene with a different robot
   * Uses the already stored customSpzData
   * @param {string} robotName - Robot to use (null for no robot)
   * @returns {Promise<string>} - Path to scene XML
   */
  async loadCustomSpzWithRobot(robotName = null) {
    if (!this.customSpzData) {
      throw new Error('No custom SPZ data stored');
    }
    console.log(`Reloading custom SPZ with robot: ${robotName}`);
    return this._setupCustomSpzScene(robotName);
  }

  /**
   * Internal helper to setup custom SPZ scene with optional robot
   * @param {string} robotName - Robot to use (null for no robot)
   * @returns {Promise<string>} - Path to scene XML
   */
  async _setupCustomSpzScene(robotName) {
    // Load basic environment XML
    const envConfig = SceneManager.ENV_CONFIGS['basic'];
    const envResponse = await fetch(envConfig.xmlPath);
    const envXml = await envResponse.text();

    // Handle uploaded robots - use their existing scene directory
    if (this.isUploadedRobot(robotName)) {
      const uploadedSceneDir = `/working/scenes/${robotName}`;

      // Check if uploaded robot files still exist
      try {
        this.mujoco.FS.readFile(`${uploadedSceneDir}/robot.xml`);
      } catch (e) {
        throw new Error('Uploaded robot files not found. Please re-upload the robot.');
      }

      // A custom SPZ has no registered environment package or task objects.
      const objectsPath = `${uploadedSceneDir}/objects.xml`;
      if (this.mujoco.FS.analyzePath(objectsPath).exists) this.mujoco.FS.unlink(objectsPath);

      // Write collision.xml if custom collision is set
      if (this.customCollisionXml) {
        this._writeToFS(`${uploadedSceneDir}/collision.xml`, this.customCollisionXml);
      }

      // Create scene XML with environment, collision, and uploaded robot
      const sceneXml = this._createSceneXmlWithCollision(envXml, 'robot', false, `custom_spz_${robotName}`, this.hasCustomCollision());
      this._writeToFS(`${uploadedSceneDir}/scene.xml`, sceneXml);

      this.currentRobot = robotName;
      this.currentEnv = 'custom_spz';
      this.scenePath = `scenes/${robotName}/scene.xml`;

      console.log(`Custom SPZ scene with uploaded robot ready: ${this.scenePath}`);
      return this.scenePath;
    }

    // Handle built-in robots
    const vfsSceneDir = `/working/scenes/custom_spz`;
    this._ensureDir('/working/scenes');
    this._ensureDir(vfsSceneDir);

    // Write collision.xml if custom collision is set
    if (this.customCollisionXml) {
      this._writeToFS(`${vfsSceneDir}/collision.xml`, this.customCollisionXml);
    }

    if (robotName && this.robotRegistry.get(robotName)) {
      const hasObjects = await this._copyRobotToDir(robotName, vfsSceneDir);
      const sceneXml = this._createSceneXmlWithCollision(envXml, robotName, hasObjects, `custom_spz_${robotName}`, this.hasCustomCollision());
      this._writeToFS(`${vfsSceneDir}/scene.xml`, sceneXml);
      this.currentRobot = robotName;
    } else {
      // No robot - still need to handle collision XML
      if (this.customCollisionXml) {
        const sceneXml = this._createSceneXmlWithCollision(envXml, null, false, 'custom_spz', true);
        this._writeToFS(`${vfsSceneDir}/scene.xml`, sceneXml);
      } else {
        this._writeToFS(`${vfsSceneDir}/scene.xml`, envXml);
      }
      this.currentRobot = null;
    }

    this.currentEnv = 'custom_spz';
    this.scenePath = `scenes/custom_spz/scene.xml`;

    console.log(`Custom SPZ scene ready: ${this.scenePath}`);
    return this.scenePath;
  }

  /**
   * Create scene XML with optional collision include
   * @param {string} envXml - Base environment XML
   * @param {string} robotName - Robot name (or null)
   * @param {boolean} hasObjects - Whether objects.xml exists
   * @param {string} sceneName - Scene name for model attribute
   * @param {boolean} hasCollision - Whether to include collision.xml
   * @returns {string} - Generated scene XML
   */
  _createSceneXmlWithCollision(envXml, robotName, hasObjects, sceneName, hasCollision) {
    // Parse environment XML
    const parser = new DOMParser();
    const doc = parser.parseFromString(envXml, 'text/xml');

    if (doc.querySelector('parsererror')) {
      throw new Error('Failed to parse environment XML');
    }

    // Update model name
    const mujoco = doc.documentElement;
    mujoco.setAttribute('model', sceneName);

    // Insert includes at the beginning (after mujoco tag)
    // Order: collision -> robot -> objects
    let insertPoint = mujoco.firstChild;

    // Add collision include if available
    if (hasCollision) {
      const collisionInclude = doc.createElement('include');
      collisionInclude.setAttribute('file', 'collision.xml');
      mujoco.insertBefore(collisionInclude, insertPoint);
      insertPoint = collisionInclude.nextSibling;
    }

    // Add robot include if specified
    if (robotName) {
      const robotInclude = doc.createElement('include');
      robotInclude.setAttribute('file', `${robotName}.xml`);
      mujoco.insertBefore(robotInclude, insertPoint);
      insertPoint = robotInclude.nextSibling;
    }

    // Objects belong to the environment and do not depend on a robot include.
    if (hasObjects) {
      const objectsInclude = doc.createElement('include');
      objectsInclude.setAttribute('file', 'objects.xml');
      mujoco.insertBefore(objectsInclude, insertPoint);
    }

    // Serialize back to string
    const serializer = new XMLSerializer();
    let result = serializer.serializeToString(doc);

    // Clean up XML
    result = result.replace(/(<\?xml[^?]*\?>)/g, '');
    result = result.replace(/\s+xmlns="[^"]*"/g, '');
    result = result.replace(/\s+xmlns:[a-z]+="[^"]*"/g, '');
    result = '<?xml version="1.0" encoding="UTF-8"?>\n' + result.trim();

    return result;
  }

  /**
   * Copy robot files (XML and meshes) to target directory.
   * @param {string} robotName - Robot name
   * @param {string} targetDir - Target VFS directory
   * @returns {Promise<void>}
   */
  async _copyRobotToDir(robotName, targetDir, spawn = null) {
    console.log(`_copyRobotToDir: robotName=${robotName}, targetDir=${targetDir}`);
    await this.robotLoader.copyPackageToScene(robotName, targetDir);
    if (spawn) this._applyRobotSpawn(`${targetDir}/${robotName}.xml`, spawn, robotName);
  }

  /** Offset every robot-owned top-level body and free-joint keyframe to a collision-free scene spawn. */
  _applyRobotSpawn(robotXmlPath, spawn, robotName) {
    const position = spawn?.position;
    if (!Array.isArray(position) || position.length !== 2 || !position.every(Number.isFinite)) {
      throw new Error(`${robotName}: invalid environment spawn position`);
    }
    const xml = this.mujoco.FS.readFile(robotXmlPath, { encoding: 'utf8' });
    const doc = new DOMParser().parseFromString(xml, 'text/xml');
    if (doc.querySelector('parsererror')) throw new Error(`${robotName}: failed to parse robot XML for spawn placement`);
    const worldbody = doc.querySelector('worldbody');
    const topLevelBodies = worldbody
      ? [...worldbody.children].filter((element) => element.tagName === 'body')
      : [];
    if (!topLevelBodies.length) throw new Error(`${robotName}: robot XML has no top-level body`);
    const yaw = Number.isFinite(spawn.yaw) ? Number(spawn.yaw) : 0;
    const yawHalf = yaw * 0.5;
    const yawQuat = [Math.cos(yawHalf), 0, 0, Math.sin(yawHalf)];
    const multiplyQuaternion = (left, right) => [
      left[0] * right[0] - left[1] * right[1] - left[2] * right[2] - left[3] * right[3],
      left[0] * right[1] + left[1] * right[0] + left[2] * right[3] - left[3] * right[2],
      left[0] * right[2] - left[1] * right[3] + left[2] * right[0] + left[3] * right[1],
      left[0] * right[3] + left[1] * right[2] - left[2] * right[1] + left[3] * right[0]
    ];
    for (const body of topLevelBodies) {
      const values = (body.getAttribute('pos') ?? '0 0 0').trim().split(/\s+/).map(Number);
      while (values.length < 3) values.push(0);
      const localX = values[0];
      const localY = values[1];
      values[0] = position[0] + Math.cos(yaw) * localX - Math.sin(yaw) * localY;
      values[1] = position[1] + Math.sin(yaw) * localX + Math.cos(yaw) * localY;
      body.setAttribute('pos', values.map((value) => Number(value.toFixed(9))).join(' '));
      const currentQuat = (body.getAttribute('quat') ?? '1 0 0 0').trim().split(/\s+/).map(Number);
      body.setAttribute('quat', multiplyQuaternion(yawQuat, currentQuat)
        .map((value) => Number(value.toFixed(9))).join(' '));
    }

    const hasFreeJoint = Boolean(doc.querySelector('freejoint, joint[type="free"]'));
    if (hasFreeJoint) {
      for (const key of doc.querySelectorAll('keyframe key[qpos]')) {
        const qpos = key.getAttribute('qpos').trim().split(/\s+/).map(Number);
        if (qpos.length >= 7) {
          const localX = qpos[0];
          const localY = qpos[1];
          qpos[0] = position[0] + Math.cos(yaw) * localX - Math.sin(yaw) * localY;
          qpos[1] = position[1] + Math.sin(yaw) * localX + Math.cos(yaw) * localY;
          qpos.splice(3, 4, ...multiplyQuaternion(yawQuat, qpos.slice(3, 7)));
          key.setAttribute('qpos', qpos.map((value) => Number(value.toFixed(9))).join(' '));
        }
      }
    }
    this._writeToFS(robotXmlPath, new XMLSerializer().serializeToString(doc));
    console.log(`Applied ${robotName} spawn: [${position.join(', ')}], yaw=${yaw}, clearance=${spawn.clearanceMeters}m`);
  }

  /**
   * List available environments
   */
  listEnvironments() {
    return Object.keys(SceneManager.ENV_CONFIGS).filter((id) => SceneManager.ENV_CONFIGS[id].visualMode);
  }

  /** Return display labels mapped to stable environment identifiers for lil-gui. */
  getEnvironmentOptions() {
    return Object.fromEntries(this.listEnvironments().map((id) => [
      SceneManager.ENV_CONFIGS[id].label ?? id,
      id
    ]));
  }

  /**
   * List available robots
   */
  listRobots() {
    return this.robotRegistry.list();
  }

  getRobot(robotName) {
    return this.robotRegistry.get(robotName);
  }

  getRobotOptions() {
    return this.robotRegistry.getOptions();
  }

  getDefaultRobot() {
    return this.robotRegistry.defaultRobot;
  }

  /**
   * Check if robot is an uploaded custom robot
   * @param {string} robotName - Robot name to check
   * @returns {boolean}
   */
  isUploadedRobot(robotName) {
    return robotName && robotName.startsWith('uploaded_');
  }

  /**
   * Reload uploaded robot scene
   * Used when reloading a scene with an uploaded robot
   * @param {string} envName - Environment name
   * @returns {Promise<string>} - Path to scene XML
   */
  async reloadUploadedRobot(envName) {
    if (!this.currentRobot || !this.isUploadedRobot(this.currentRobot)) {
      throw new Error('No uploaded robot to reload');
    }

    const vfsSceneDir = `/working/scenes/${this.currentRobot}`;

    // Check if scene files still exist
    try {
      this.mujoco.FS.readFile(`${vfsSceneDir}/scene.xml`);
    } catch (e) {
      throw new Error('Uploaded robot scene files not found. Please re-upload the robot.');
    }

    // If environment changed, recreate scene XML with new environment
    const effectiveEnv = (envName === 'custom_spz') ? 'basic' : envName;
    const envConfig = SceneManager.ENV_CONFIGS[effectiveEnv];
    if (!envConfig) {
      throw new Error(`Unknown environment: ${envName}`);
    }

    const envResponse = await fetch(envConfig.xmlPath);
    const envXml = await envResponse.text();

    const objectsPath = `${vfsSceneDir}/objects.xml`;
    if (this.mujoco.FS.analyzePath(objectsPath).exists) this.mujoco.FS.unlink(objectsPath);
    const hasObjects = await this._copyEnvironmentToDir(envConfig, vfsSceneDir);

    // Recreate scene XML with environment
    const sceneName = `${effectiveEnv}_${this.currentRobot}`;
    const sceneXml = this._createSceneXml(envXml, 'robot', hasObjects, sceneName);
    this._writeToFS(`${vfsSceneDir}/scene.xml`, sceneXml);

    this.currentEnv = envName;
    this.scenePath = `scenes/${this.currentRobot}/scene.xml`;

    return this.scenePath;
  }
}

// Singleton instance
let sceneManagerInstance = null;

export function getSceneManager(mujoco) {
  if (!sceneManagerInstance && mujoco) {
    sceneManagerInstance = new SceneManager(mujoco);
  }
  return sceneManagerInstance;
}
