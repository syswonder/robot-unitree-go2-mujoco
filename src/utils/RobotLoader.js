const TEXT_EXTENSIONS = new Set(['xml', 'mjcf', 'urdf', 'json', 'txt']);

function normalizeRelativePath(file, robotId) {
  const normalized = file.replace(/\\/g, '/').replace(/^\.\//, '');
  if (!normalized || normalized.startsWith('/') || normalized.split('/').includes('..')) {
    throw new Error(`${robotId}: invalid package file path: ${file}`);
  }
  return normalized;
}

/** Load built-in robot packages and uploaded robot folders into MuJoCo's VFS. */
export class RobotLoader {
  constructor(mujoco, registry) {
    this.mujoco = mujoco;
    this.registry = registry;
    this.loadedRobots = new Set();
  }

  isPackageLoaded(robotId) {
    return this.loadedRobots.has(robotId);
  }

  /** Download every simulation file declared by a robot package exactly once. */
  async ensurePackageLoaded(robotId) {
    if (this.loadedRobots.has(robotId)) return;
    const robot = this.registry.get(robotId);
    if (!robot) throw new Error(`Unknown robot: ${robotId}`);

    const filesResponse = await fetch(robot.filesPath);
    if (!filesResponse.ok) throw new Error(`${robotId}: simulation file index not found`);
    const files = await filesResponse.json();
    if (!Array.isArray(files) || !files.length) throw new Error(`${robotId}: simulation file index is empty`);
    const normalizedFiles = files.map((file) => normalizeRelativePath(file, robotId));
    for (const required of [robot.model]) {
      if (!normalizedFiles.includes(required)) throw new Error(`${robotId}: file index is missing ${required}`);
    }

    this._ensureDirectory('/working/robots');
    this._ensureDirectory(`/working/robots/${robotId}`);
    const downloaded = await Promise.all(normalizedFiles.map(async (file) => {
      const response = await fetch(`${robot.packagePath}/${file}`);
      if (!response.ok) throw new Error(`${robotId}: failed to fetch ${file} (${response.status})`);
      const extension = file.split('.').pop().toLowerCase();
      const data = TEXT_EXTENSIONS.has(extension) ?
        await response.text() : new Uint8Array(await response.arrayBuffer());
      return { file, data };
    }));

    for (const { file, data } of downloaded) {
      this._ensureParentDirectories(`/working/robots/${robotId}`, file);
      this.mujoco.FS.writeFile(`/working/robots/${robotId}/${file}`, data);
    }
    this.loadedRobots.add(robotId);
    console.log(`Robot package loaded: ${robotId} (${downloaded.length} files)`);
  }

  /** Copy a complete package into a scene directory and normalize include filenames. */
  async copyPackageToScene(robotId, targetDir) {
    const robot = this.registry.get(robotId);
    if (!robot) throw new Error(`Unknown robot: ${robotId}`);
    await this.ensurePackageLoaded(robotId);

    const files = await (await fetch(robot.filesPath)).json();
    for (const rawFile of files) {
      const file = normalizeRelativePath(rawFile, robotId);
      const source = `/working/robots/${robotId}/${file}`;
      const destinationFile = file === robot.model ? `${robotId}.xml` : file;
      this._ensureParentDirectories(targetDir, destinationFile);
      this.mujoco.FS.writeFile(`${targetDir}/${destinationFile}`, this.mujoco.FS.readFile(source));
    }
    const path = `${targetDir}/${robotId}.xml`;
    this.mujoco.FS.writeFile(path, resolveRobotAssetPaths(this.mujoco.FS.readFile(path, { encoding: 'utf8' }), targetDir));
  }

  /** Parse one user-selected folder while retaining the existing upload behavior. */
  async loadUploadedRobot(files) {
    const meshFiles = new Map();
    let robotXml = null;
    let robotName = null;
    for (const file of files) {
      const path = file.webkitRelativePath || file.name;
      const pathParts = path.split('/');
      if (pathParts.length > 1 && !robotName) robotName = pathParts[0];
      if (path.endsWith('.xml')) {
        const content = await file.text();
        const fileName = path.split('/').pop().toLowerCase();
        if (!fileName.includes('object') && !robotXml) robotXml = content;
      } else if (path.includes('assets/') || path.includes('meshes/') || path.includes('mesh/')) {
        const meshName = path.split(/(?:assets|meshes?)\//).pop();
        meshFiles.set(meshName, await file.arrayBuffer());
      }
    }
    if (!robotXml) throw new Error('No robot XML file found in uploaded folder');
    return { robotXml, meshFiles, robotName: robotName ?? 'user_robot' };
  }

  _ensureDirectory(path) {
    if (!this.mujoco.FS.analyzePath(path).exists) this.mujoco.FS.mkdir(path);
  }

  /** Create all intermediate directories below a known existing root. */
  _ensureParentDirectories(root, relativeFile) {
    let current = root;
    this._ensureDirectory(current);
    for (const part of relativeFile.split('/').slice(0, -1)) {
      current += `/${part}`;
      this._ensureDirectory(current);
    }
  }
}

/** Resolve robot assets in the virtual copy so environment compiler directories cannot redirect them. */
export function resolveRobotAssetPaths(xml, targetDir) {
  const doc = new DOMParser().parseFromString(xml, 'text/xml');
  if (doc.querySelector('parsererror')) throw new Error('Invalid Go2 XML');
  const compiler = doc.querySelector('compiler');
  for (const type of ['mesh', 'texture']) {
    const directory = compiler?.getAttribute(`${type}dir`) ?? '';
    for (const asset of doc.querySelectorAll(`asset > ${type}[file]`)) {
      const file = asset.getAttribute('file');
      if (!file.startsWith('/')) asset.setAttribute('file', `${targetDir}/${directory ? `${directory}/` : ''}${file}`);
    }
    compiler?.removeAttribute(`${type}dir`);
  }
  return new XMLSerializer().serializeToString(doc);
}
