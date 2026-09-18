import * as THREE from 'three';
import { Reflector  } from './utils/Reflector.js';
import { MuJoCoDemo } from './main.js';
import { keyboardController } from './utils/KeyboardControl.js';
import { getSceneManager } from './utils/SceneManager.js';
import { createMujocoMeshGeometry, createMujocoColorTexture } from './utils/MujocoGeometry.js';

/**
 * Load a modular scene (environment + robot + objects)
 * @param {MuJoCoDemo} context - Demo context
 * @param {string} envName - Environment identifier from the generated manifest
 * @param {string} robotName - Robot name (e.g., 'xlerobot', 'panda', 'g1')
 */
export async function loadModularScene(context, envName, robotName) {
  const sceneManager = getSceneManager(context.mujoco);

  // Load modular scene - this creates merged XML
  const scenePath = await sceneManager.loadModularScene(envName, robotName);

  // Update params for compatibility
  context.params.scene = scenePath;
  context.params.robot = robotName;
  context.params.environment = envName;

  // Load the merged scene using existing loading function
  [context.model, context.data, context.bodies, context.lights] =
    await loadSceneFromURL(context.mujoco, scenePath, context);

  context.mujoco.mj_forward(context.model, context.data);

  // Run update callbacks (support async)
  for (let i = 0; i < context.updateGUICallbacks.length; i++) {
    await context.updateGUICallbacks[i](context.model, context.data, context.params);
  }

  return sceneManager;
}

export async function reloadFunc() {
  // Reload the current scene
  const sceneManager = getSceneManager(this.mujoco);
  const envName = this.params.environment || sceneManager.getDefaultEnvironment();
  const robotName = this.params.robot || sceneManager.getDefaultRobot();

  // Handle custom SPZ mode
  if (sceneManager.hasCustomSpz() && envName === 'custom_spz') {
    await sceneManager.loadCustomSpzWithRobot(robotName);
  } else if (sceneManager.isUploadedRobot(robotName)) {
    // Handle uploaded robot reload
    await sceneManager.reloadUploadedRobot(envName);
  } else {
    // Normal robot reload
    await sceneManager.loadModularScene(envName, robotName);
  }

  this.params.scene = sceneManager.scenePath;
  [this.model, this.data, this.bodies, this.lights] =
    await loadSceneFromURL(this.mujoco, sceneManager.scenePath, this);
  this.mujoco.mj_forward(this.model, this.data);

  // Run update callbacks (support async)
  for (let i = 0; i < this.updateGUICallbacks.length; i++) {
    await this.updateGUICallbacks[i](this.model, this.data, this.params);
  }
}

/** @param {MuJoCoDemo} parentContext*/
export async function setupGUI(parentContext) {

  const developerMode = new URLSearchParams(window.location.search).get('dev') === '1';

  // Make sure we reset the camera when the scene is changed or reloaded.
  parentContext.updateGUICallbacks.length = 0;
  parentContext.updateGUICallbacks.push((model, data, params) => {
    resetDefaultCamera(parentContext);
  });

  // Initialize modular scene params
  const initializedSceneManager = getSceneManager(parentContext.mujoco);
  parentContext.params.environment = parentContext.params.environment || initializedSceneManager.getDefaultEnvironment();
  parentContext.params.robot = parentContext.params.robot || initializedSceneManager.getDefaultRobot();

  const configureBodyDragging = (model, data, params) => {
    const robot = initializedSceneManager.getRobot(params.robot);
    parentContext.dragStateManager.configure(model, robot?.dragRootBody);
  };
  configureBodyDragging(parentContext.model, parentContext.data, parentContext.params);
  parentContext.updateGUICallbacks.push(configureBodyDragging);

  // Helper to load scene and run callbacks
  const loadSceneAndUpdate = async (scenePath, robotName) => {
    parentContext.params.scene = scenePath;
    [parentContext.model, parentContext.data, parentContext.bodies, parentContext.lights] =
      await loadSceneFromURL(parentContext.mujoco, scenePath, parentContext);
    parentContext.mujoco.mj_forward(parentContext.model, parentContext.data);

    // Run update callbacks (support async)
    for (let i = 0; i < parentContext.updateGUICallbacks.length; i++) {
      await parentContext.updateGUICallbacks[i](parentContext.model, parentContext.data, parentContext.params);
    }

    // Setup keyboard control (async)
    const robot = initializedSceneManager.getRobot(robotName);
    if (keyboardController.hasConfig(robot)) {
      await keyboardController.enable(
        robot,
        parentContext.model,
        parentContext.data,
        parentContext.mujoco,
        { keyboardInputEnabled: developerMode }
      );
    }
  };

  // Helper to load custom SPZ 3DGS
  const loadCustomSpz3DGS = async (spzData) => {
    const blob = new Blob([spzData], { type: 'application/octet-stream' });
    const blobUrl = URL.createObjectURL(blob);

    if (parentContext.gsController && parentContext.gsController.enabled) {
      parentContext.gsController.disable();
    }
    if (parentContext.gsController) {
      await parentContext.gsController.enable(blobUrl);
    }
  };

  parentContext.syncEnvironmentVisual = async () => {
    if (!parentContext.gsController) return;
    if (parentContext.gsController.enabled) parentContext.gsController.disable();
    if (initializedSceneManager.getVisualMode(parentContext.params.environment) !== 'spz') return;
    const spzPath = initializedSceneManager.getSpzPath();
    const transformMetadata = await initializedSceneManager.getSpzTransform();
    await parentContext.gsController.enable(spzPath, transformMetadata);
  };

  // Loading overlay helper
  let loadingOverlay = null;
  const showLoading = (message = 'Loading...') => {
    if (!loadingOverlay) {
      loadingOverlay = document.createElement('div');
      loadingOverlay.style.cssText = `
        position: fixed;
        top: 0;
        left: 0;
        width: 100%;
        height: 100%;
        background: rgba(0, 0, 0, 0.7);
        display: flex;
        flex-direction: column;
        justify-content: center;
        align-items: center;
        z-index: 9999;
        pointer-events: auto;
      `;

      const spinner = document.createElement('div');
      spinner.style.cssText = `
        width: 50px;
        height: 50px;
        border: 4px solid rgba(255, 255, 255, 0.3);
        border-top: 4px solid #fff;
        border-radius: 50%;
        animation: spin 1s linear infinite;
      `;

      const style = document.createElement('style');
      style.textContent = `
        @keyframes spin {
          0% { transform: rotate(0deg); }
          100% { transform: rotate(360deg); }
        }
      `;
      document.head.appendChild(style);

      const text = document.createElement('div');
      text.id = 'loading-text';
      text.style.cssText = `
        color: white;
        font-size: 18px;
        margin-top: 20px;
        font-family: Arial, sans-serif;
      `;
      text.textContent = message;

      loadingOverlay.appendChild(spinner);
      loadingOverlay.appendChild(text);
    } else {
      const text = loadingOverlay.querySelector('#loading-text');
      if (text) text.textContent = message;
    }

    if (!document.body.contains(loadingOverlay)) {
      document.body.appendChild(loadingOverlay);
    }
  };

  const hideLoading = () => {
    if (loadingOverlay && document.body.contains(loadingOverlay)) {
      document.body.removeChild(loadingOverlay);
    }
  };

  // Load one captured selection. Requests are serialized below because both
  // MuJoCo's VFS and the renderer are shared mutable resources.
  const loadSceneSelection = async (envName, robotName) => {
    const sceneManager = getSceneManager(parentContext.mujoco);
    parentContext.params.environment = envName;
    parentContext.params.robot = robotName;

    showLoading(`Loading ${robotName}...`);

    try {
      // Custom SPZ mode: reload with new robot
      if (sceneManager.hasCustomSpz() && envName === 'custom_spz') {
        await sceneManager.loadCustomSpzWithRobot(robotName);
        await loadSceneAndUpdate(sceneManager.scenePath, robotName);
        await loadCustomSpz3DGS(sceneManager.getCustomSpzData());
      } else {
        // Clear custom SPZ if switching to a normal environment
        if (sceneManager.hasCustomSpz()) {
          sceneManager.clearCustomSpz();
          if (parentContext.gsController && parentContext.gsController.enabled) {
            parentContext.gsController.disable();
          }
        }

        // Handle uploaded robot or normal robot
        if (sceneManager.isUploadedRobot(robotName)) {
          await sceneManager.reloadUploadedRobot(envName);
          await loadSceneAndUpdate(sceneManager.scenePath, robotName);
        } else {
          await loadModularScene(parentContext, envName, robotName);
        }

        await parentContext.syncEnvironmentVisual();
      }
    } finally {
      hideLoading();
    }
  };

  let sceneSwitchQueue = Promise.resolve();
  const requestSceneLoad = (envName = parentContext.params.environment,
    robotName = parentContext.params.robot) => {
    const operation = sceneSwitchQueue
      .catch(() => {})
      .then(() => loadSceneSelection(envName, robotName));
    sceneSwitchQueue = operation;
    return operation;
  };

  const environmentController = parentContext.gui.add(
    parentContext.params,
    'environment',
    initializedSceneManager.getEnvironmentOptions()
  ).name('Environment');

  const navigateToEnvironment = (envName) => {
    if (envName === initializedSceneManager.currentEnv) return false;
    showLoading(`Loading ${parentContext.params.robot}...`);
    const url = new URL(window.location.href);
    url.searchParams.set('environment', envName);
    url.searchParams.set('robot', parentContext.params.robot);
    window.location.assign(url);
    return true;
  };

  environmentController.onChange(navigateToEnvironment);

  parentContext.switchEnvironment = (envName) => {
    if (!initializedSceneManager.listEnvironments().includes(envName)) {
      throw new Error(`Unknown environment: ${envName}`);
    }
    parentContext.params.environment = envName;
    environmentController.updateDisplay();
    return navigateToEnvironment(envName);
  };

  // Add robot selection dropdown
  const robotController = parentContext.gui.add(
    parentContext.params,
    'robot',
    initializedSceneManager.getRobotOptions()
  ).name('Robot').onChange((robotName) => {
    requestSceneLoad(parentContext.params.environment, robotName).catch((error) => {
      console.error('Robot switch failed:', error);
    });
  });
  if (!developerMode) robotController.hide();

  // Add upload robot button
  const uploadRobotBtn = {
    uploadRobot: () => {
      const input = document.createElement('input');
      input.type = 'file';
      input.webkitdirectory = true;
      input.multiple = true;
      input.onchange = async (e) => {
        const files = e.target.files;
        if (files.length === 0) return;

        try {
          console.log('Uploading robot folder with', files.length, 'files');

          const sceneManager = getSceneManager(parentContext.mujoco);
          const scenePath = await sceneManager.loadUploadedRobot(files, parentContext.params.environment);
          parentContext.params.robot = sceneManager.currentRobot;

          await loadSceneAndUpdate(scenePath, sceneManager.currentRobot);

          // If in custom SPZ mode, reload the 3DGS
          if (sceneManager.hasCustomSpz()) {
            await loadCustomSpz3DGS(sceneManager.getCustomSpzData());
          }

          console.log('Custom robot loaded successfully');
        } catch (err) {
          console.error('Failed to load uploaded robot:', err);
          alert('Failed to load robot: ' + err.message);
        }
      };
      input.click();
    }
  };
  const uploadRobotController = parentContext.gui.add(uploadRobotBtn, 'uploadRobot')
    .name('Upload Robot Folder');
  if (!developerMode) uploadRobotController.hide();

  // Add upload SPZ button for custom 3DGS scenes
  const uploadSpzBtn = {
    uploadSpz: () => {
      const input = document.createElement('input');
      input.type = 'file';
      input.accept = '.spz';
      input.onchange = async (e) => {
        const file = e.target.files[0];
        if (!file) return;

        try {
          console.log('Uploading SPZ file:', file.name);

          const sceneManager = getSceneManager(parentContext.mujoco);
          const robotName = parentContext.params.robot;
          const scenePath = await sceneManager.loadCustomSpz(file, robotName);

          parentContext.params.environment = 'custom_spz';
          await loadSceneAndUpdate(scenePath, robotName);

          if (sceneManager.hasCustomSpz()) {
            await loadCustomSpz3DGS(sceneManager.getCustomSpzData());
            console.log('Custom 3DGS loaded from uploaded SPZ');
          }

          console.log('Custom SPZ scene loaded successfully');
        } catch (err) {
          console.error('Failed to load SPZ file:', err);
          alert('Failed to load SPZ: ' + err.message);
        }
      };
      input.click();
    }
  };
  const uploadSpzController = parentContext.gui.add(uploadSpzBtn, 'uploadSpz')
    .name('Upload 3DGS (.spz)');
  if (!developerMode) uploadSpzController.hide();

  // Add upload collision XML button for custom SPZ scenes
  const uploadCollisionBtn = {
    uploadCollision: () => {
      const input = document.createElement('input');
      input.type = 'file';
      input.accept = '.xml';
      input.onchange = async (e) => {
        const file = e.target.files[0];
        if (!file) return;

        try {
          console.log('Uploading collision XML file:', file.name);
          const sceneManager = getSceneManager(parentContext.mujoco);

          // Read the XML content
          const xmlContent = await file.text();
          sceneManager.setCustomCollision(xmlContent);

          // If already in custom_spz mode, reload scene with collision
          if (sceneManager.hasCustomSpz()) {
            const robotName = parentContext.params.robot;
            const scenePath = await sceneManager.loadCustomSpzWithRobot(robotName);
            await loadSceneAndUpdate(scenePath, robotName);
            await loadCustomSpz3DGS(sceneManager.getCustomSpzData());
            console.log('Scene reloaded with collision XML');
          } else {
            console.log('Collision XML stored. Upload a SPZ file to use it.');
            alert('Collision XML loaded. Now upload a SPZ file to create the scene.');
          }
        } catch (err) {
          console.error('Failed to load collision XML:', err);
          alert('Failed to load collision XML: ' + err.message);
        }
      };
      input.click();
    }
  };
  const uploadCollisionController = parentContext.gui.add(uploadCollisionBtn, 'uploadCollision')
    .name('Upload Collision (.xml)');
  if (!developerMode) uploadCollisionController.hide();

  // Add a help menu.
  // Parameters:
  //  Name: "Help".
  //  When pressed, a help menu is displayed in the top left corner. When pressed again
  //  the help menu is removed.
  //  Can also be triggered by pressing F1.
  // Has a dark transparent background.
  // Has two columns: one for putting the action description, and one for the action key trigger.keyframeNumber
  let keyInnerHTML = '';
  let actionInnerHTML = '';
  const displayHelpMenu = () => {
    if (parentContext.params.help) {
      const helpMenu = document.createElement('div');
      helpMenu.style.position = 'absolute';
      helpMenu.style.top = '10px';
      helpMenu.style.left = '10px';
      helpMenu.style.color = 'white';
      helpMenu.style.font = 'normal 18px Arial';
      helpMenu.style.backgroundColor = 'rgba(0, 0, 0, 0.5)';
      helpMenu.style.padding = '10px';
      helpMenu.style.borderRadius = '10px';
      helpMenu.style.display = 'flex';
      helpMenu.style.flexDirection = 'column';
      helpMenu.style.alignItems = 'center';
      helpMenu.style.justifyContent = 'center';
      helpMenu.style.width = '400px';
      helpMenu.style.height = '400px';
      helpMenu.style.overflow = 'auto';
      helpMenu.style.zIndex = '1000';

      const helpMenuTitle = document.createElement('div');
      helpMenuTitle.style.font = 'bold 24px Arial';
      helpMenuTitle.innerHTML = '';
      helpMenu.appendChild(helpMenuTitle);

      const helpMenuTable = document.createElement('table');
      helpMenuTable.style.width = '100%';
      helpMenuTable.style.marginTop = '10px';
      helpMenu.appendChild(helpMenuTable);

      const helpMenuTableBody = document.createElement('tbody');
      helpMenuTable.appendChild(helpMenuTableBody);

      const helpMenuRow = document.createElement('tr');
      helpMenuTableBody.appendChild(helpMenuRow);

      const helpMenuActionColumn = document.createElement('td');
      helpMenuActionColumn.style.width = '50%';
      helpMenuActionColumn.style.textAlign = 'right';
      helpMenuActionColumn.style.paddingRight = '10px';
      helpMenuRow.appendChild(helpMenuActionColumn);

      const helpMenuKeyColumn = document.createElement('td');
      helpMenuKeyColumn.style.width = '50%';
      helpMenuKeyColumn.style.textAlign = 'left';
      helpMenuKeyColumn.style.paddingLeft = '10px';
      helpMenuRow.appendChild(helpMenuKeyColumn);

      const helpMenuActionText = document.createElement('div');
      helpMenuActionText.innerHTML = actionInnerHTML;
      helpMenuActionColumn.appendChild(helpMenuActionText);

      const helpMenuKeyText = document.createElement('div');
      helpMenuKeyText.innerHTML = keyInnerHTML;
      helpMenuKeyColumn.appendChild(helpMenuKeyText);

      // Close buttom in the top.
      const helpMenuCloseButton = document.createElement('button');
      helpMenuCloseButton.innerHTML = 'Close';
      helpMenuCloseButton.style.position = 'absolute';
      helpMenuCloseButton.style.top = '10px';
      helpMenuCloseButton.style.right = '10px';
      helpMenuCloseButton.style.zIndex = '1001';
      helpMenuCloseButton.onclick = () => {
        helpMenu.remove();
      };
      helpMenu.appendChild(helpMenuCloseButton);

      document.body.appendChild(helpMenu);
    } else {
      document.body.removeChild(document.body.lastChild);
    }
  }
  document.addEventListener('keydown', (event) => {
    if (event.key === 'F1') {
      parentContext.params.help = !parentContext.params.help;
      displayHelpMenu();
      event.preventDefault();
    }
  });
  keyInnerHTML += 'F1<br>';
  actionInnerHTML += 'Help<br>';

  let simulationFolder = parentContext.gui.addFolder("Simulation");

  // Add pause simulation checkbox.
  // Parameters:
  //  Under "Simulation" folder.
  //  Name: "Pause Simulation".
  //  When paused, a "pause" text in white is displayed in the top left corner.
  const pauseSimulation = simulationFolder.add(parentContext.params, 'paused').name('Pause Simulation');
  pauseSimulation.onChange((value) => {
    if (value) {
      const pausedText = document.createElement('div');
      pausedText.style.position = 'absolute';
      pausedText.style.top = '10px';
      pausedText.style.left = '10px';
      pausedText.style.color = 'white';
      pausedText.style.font = 'normal 18px Arial';
      pausedText.innerHTML = 'pause';
      parentContext.container.appendChild(pausedText);
    } else {
      parentContext.container.removeChild(parentContext.container.lastChild);
    }
  });

  // Add reload model button.
  // Parameters:
  //  Under "Simulation" folder.
  //  Name: "Reload".
  //  When pressed, calls the reload function.
  //  Can also be triggered by pressing ctrl + L.
  simulationFolder.add({reload: () => { reload(); }}, 'reload').name('Reload');
  document.addEventListener('keydown', (event) => {
    if (event.ctrlKey && event.code === 'KeyL') { reload();  event.preventDefault(); }});
  actionInnerHTML += 'Reload XML<br>';
  keyInnerHTML += 'Ctrl L<br>';

  // Add reset simulation button.
  // Parameters:
  //  Under "Simulation" folder.
  //  Name: "Reset".
  //  When pressed, resets the simulation to the initial state.
  //  Can also be triggered by pressing backspace.
  const resetSimulation = () => {
    keyboardController.resetRobot();
  };
  simulationFolder.add({reset: () => { resetSimulation(); }}, 'reset').name('Reset');
  document.addEventListener('keydown', (event) => {
    if (['INPUT', 'TEXTAREA', 'SELECT'].includes(event.target.tagName) || event.target.isContentEditable) return;
    if (event.code === 'Backspace') { resetSimulation(); event.preventDefault(); }});
  actionInnerHTML += 'Reset simulation<br>';
  keyInnerHTML += 'Backspace<br>';

  // Add keyframe slider.
  let nkeys = parentContext.model.nkey;
  let keyframeGUI = simulationFolder.add(parentContext.params, "keyframeNumber", 0, nkeys - 1, 1).name('Load Keyframe').listen();
  keyframeGUI.onChange((value) => {
    if (value < parentContext.model.nkey) {
      parentContext.data.qpos.set(parentContext.model.key_qpos.slice(
        value * parentContext.model.nq, (value + 1) * parentContext.model.nq)); }});
  parentContext.updateGUICallbacks.push((model, data, params) => {
    let nkeys = parentContext.model.nkey;
    console.log("new model loaded. has " + nkeys + " keyframes.");
    if (nkeys > 0) {
      keyframeGUI.max(nkeys - 1);
      keyframeGUI.domElement.style.opacity = 1.0;
    } else {
      // Disable keyframe slider if no keyframes are available.
      keyframeGUI.max(0);
      keyframeGUI.domElement.style.opacity = 0.5;
    }
  });

  // Add sliders for ctrlnoiserate and ctrlnoisestd; min = 0, max = 2, step = 0.01.
  simulationFolder.add(parentContext.params, 'ctrlnoiserate', 0.0, 2.0, 0.01).name('Noise rate' );
  simulationFolder.add(parentContext.params, 'ctrlnoisestd' , 0.0, 2.0, 0.01).name('Noise scale');

  let textDecoder = new TextDecoder("utf-8");
  let nullChar    = textDecoder.decode(new ArrayBuffer(1));

  // Add actuator sliders.
  let actuatorFolder = simulationFolder.addFolder("Actuators");
  if (!developerMode) actuatorFolder.hide();
  
  // Package metadata identifies actuators owned by a keyboard controller.
  const addActuators = (model, data, params) => {
    let act_range = model.actuator_ctrlrange;
    let actuatorGUIs = [];
    const robot = initializedSceneManager.getRobot(params.robot);
    const ikControlledJoints = robot?.ikActuators ?? [];
    const driverControlledJoints = robot?.controlledActuators ?? [];
    for (let i = 0; i < model.nu; i++) {
      if (!model.actuator_ctrllimited[i]) { continue; }
      let name = textDecoder.decode(
        parentContext.model.names.subarray(
          parentContext.model.name_actuatoradr[i])).split(nullChar)[0];

      parentContext.params[name] = 0.0;
      
      // Use step size of 0.01 for better precision control
      let actuatorGUI = actuatorFolder.add(parentContext.params, name, act_range[2 * i], act_range[2 * i + 1], 0.01).name(name).listen();
      
      const isIKControlled = ikControlledJoints.includes(name);
      const isDriverControlled = driverControlledJoints.includes(name);
      const isControllerManaged = isIKControlled || isDriverControlled;
      if (isIKControlled) {
        actuatorGUI.name(name + ' (IK)');
      } else if (isDriverControlled) {
        actuatorGUI.name(name + ' (Driver)');
      }
      
      actuatorGUIs.push(actuatorGUI);
      actuatorGUI.onChange((value) => {
        // Package controllers own these values and overwrite them every control step.
        if (isControllerManaged) {
          return;
        }
        // Round to 2 decimal places for display and control
        const rounded = Math.round(value * 100) / 100;
        parentContext.params[name] = rounded;
        data.ctrl[i] = rounded;
      });
      
      // Disable after onChange to ensure it's applied
      if (isControllerManaged) {
        actuatorGUI.disable();
      }
    }
    return actuatorGUIs;
  };
  let actuatorGUIs = addActuators(parentContext.model, parentContext.data, parentContext.params);
  parentContext.updateGUICallbacks.push((model, data, params) => {
    for (let i = 0; i < actuatorGUIs.length; i++) {
      actuatorGUIs[i].destroy();
    }
    actuatorGUIs = addActuators(model, data, params);
  });
  actuatorFolder.close();

  // Robot package sensors share one stable panel and are reconfigured on reload.
  const sensorFolder = simulationFolder.addFolder('Sensors');
  const sensorUI = {
    showLidar: false,
    liveCamera: '',
    liveRgba: false
  };
  const sensorStatus = parentContext.sensorSuite.status;
  sensorFolder.add(sensorStatus, 'lidar').name('LiDAR').listen().disable();
  sensorFolder.add(sensorStatus, 'imu').name('IMU').listen().disable();
  sensorFolder.add(sensorStatus, 'camera').name('Camera').listen().disable();
  sensorFolder.add(sensorUI, 'showLidar').name('Show LiDAR points').onChange((visible) => {
    parentContext.sensorSuite.setLidarVisible(visible);
  });
  let sensorCameraGUIs = [];

  const setupRobotSensors = (model, data, params) => {
    for (const controller of sensorCameraGUIs) controller.destroy();
    sensorCameraGUIs = [];
    const robot = initializedSceneManager.getRobot(params.robot);
    const available = parentContext.sensorSuite.configure(robot, model, data);
    sensorFolder.domElement.style.display = available ? '' : 'none';
    sensorUI.showLidar = false;
    sensorUI.liveRgba = false;
    if (!available) return;

    const cameras = robot.sensors.cameras ?? [];
    if (cameras.length) {
      const cameraOptions = Object.fromEntries(cameras.map((camera) => [camera.label, camera.id]));
      sensorUI.liveCamera = cameras[0].id;
      let liveToggle = null;
      parentContext.sensorSuite.onLiveCameraChange = (active) => {
        sensorUI.liveRgba = active;
        liveToggle?.updateDisplay();
      };
      sensorCameraGUIs.push(sensorFolder.add(sensorUI, 'liveCamera', cameraOptions)
        .name('Live camera').onChange((cameraId) => {
          if (sensorUI.liveRgba) parentContext.sensorSuite.startLiveCamera(cameraId);
        }));
      liveToggle = sensorFolder.add(sensorUI, 'liveRgba').name('Live RGBA view').listen().onChange((enabled) => {
        if (enabled) {
          parentContext.sensorSuite.startLiveCamera(sensorUI.liveCamera);
        } else {
          parentContext.sensorSuite.stopLiveCamera();
        }
      });
      sensorCameraGUIs.push(liveToggle);
    }

    for (const camera of cameras) {
      const action = {
        capture: () => {
          try {
            const frame = parentContext.sensorSuite.captureCamera(camera.id);
            parentContext.sensorSuite.showCameraPreview(frame);
          } catch (error) {
            console.error(`Failed to capture ${camera.id}:`, error);
          }
        }
      };
      sensorCameraGUIs.push(sensorFolder.add(action, 'capture').name(`Capture ${camera.label}`));
    }
  };

  setupRobotSensors(parentContext.model, parentContext.data, parentContext.params);
  parentContext.updateGUICallbacks.push(setupRobotSensors);
  sensorFolder.close();

  // Match MuJoCo-GS-Web's right-side policy folder and explicit load/disable flow.
  const policyFolder = simulationFolder.addFolder('Policy');
  policyFolder.domElement.id = 'policy-folder';
  const policyUI = {
    strategy: '',
    status: 'Not Loaded',
    toggle: async () => {}
  };
  let policyPending = false;
  let policyStrategyController = null;
  const policyStatusController = policyFolder.add(policyUI, 'status').name('Status').listen().disable();
  policyStatusController.domElement.id = 'policy-status';

  const getPolicies = (robotName = parentContext.params.robot) =>
    initializedSceneManager.getRobot(robotName)?.policy?.options ?? [];

  /** Reflect the controller's acknowledged policy state in the right-side GUI. */
  const refreshPolicyStatus = () => {
    const policy = keyboardController.getPolicyStatus();
    if (!policyPending) {
      policyUI.status = !getPolicies().length ? 'Unavailable' :
        policy.error ? `Error: ${policy.error}` : policy.loaded ? 'Running' : 'Not Loaded';
    }
    policyStatusController.updateDisplay();
  };

  /** Rebuild the policy selector when robot metadata changes. */
  const updatePolicyStrategies = (robotName = parentContext.params.robot) => {
    const robot = initializedSceneManager.getRobot(robotName);
    const policies = getPolicies(robotName);
    const requested = new URLSearchParams(window.location.search).get('policy');
    const current = policies.find((policy) => policy.id === policyUI.strategy);
    const selected = policies.find((policy) => policy.id === requested) ?? current ??
      policies.find((policy) => policy.id === robot?.policy?.id) ?? policies[0];
    policyUI.strategy = selected?.id ?? '';
    policyStrategyController?.destroy();
    const options = Object.fromEntries(policies.map((policy) => [policy.label, policy.id]));
    policyStrategyController = policyFolder.add(policyUI, 'strategy', options).name('Strategy');
    policyStrategyController.domElement.id = 'policy-strategy';
    policyStrategyController.onChange(async () => {
      try {
        const active = keyboardController.getPolicyStatus();
        if (active.loaded) {
          await keyboardController.policyCommand({
            type: 'policy', action: 'unload', id: active.id ?? 'moe_rough'
          });
        }
        refreshPolicyStatus();
      } catch (error) {
        policyUI.status = `Error: ${error.message ?? error}`;
        policyStatusController.updateDisplay();
      }
    });
  };

  /** Serialize explicit policy activation and deactivation without resetting simulation pose. */
  policyUI.toggle = async () => {
    if (policyPending) return;
    const selected = getPolicies().find((policy) => policy.id === policyUI.strategy);
    if (!selected) {
      policyUI.status = 'Unavailable';
      policyStatusController.updateDisplay();
      return;
    }
    const loaded = keyboardController.getPolicyStatus().loaded;
    policyPending = true;
    policyUI.status = loaded ? 'Disabling...' : 'Loading...';
    policyStatusController.updateDisplay();
    policyToggleController.disable();
    policyStrategyController?.disable();
    try {
      await keyboardController.policyCommand({
        type: 'policy',
        action: loaded ? 'unload' : 'load',
        id: selected.id
      });
    } catch (error) {
      policyUI.status = `Error: ${error.message ?? error}`;
      policyStatusController.updateDisplay();
    } finally {
      policyPending = false;
      policyToggleController.enable();
      policyStrategyController?.enable();
      refreshPolicyStatus();
    }
  };

  updatePolicyStrategies();
  const policyToggleController = policyFolder.add(policyUI, 'toggle').name('Load / Disable Policy');
  policyToggleController.domElement.id = 'policy-toggle';
  policyFolder.close();
  const policyStatusTimer = window.setInterval(refreshPolicyStatus, 500);
  window.addEventListener('pagehide', () => window.clearInterval(policyStatusTimer), { once: true });
  parentContext.updateGUICallbacks.push((_model, _data, params) => {
    updatePolicyStrategies(params.robot);
    refreshPolicyStatus();
  });

  // Add Keyboard Controls folder (only shown for scenes with keyboard config)
  let keyboardFolder = null;
  let keyboardLabel = null;

  const setupKeyboardControls = async (model, data, params) => {
    // Remove existing folder if any
    if (keyboardFolder) {
      keyboardFolder.destroy();
      keyboardFolder = null;
      keyboardLabel = null;
    }

    // Check if current robot has keyboard control config
    const robot = initializedSceneManager.getRobot(params.robot);
    if (keyboardController.hasConfig(robot)) {
      await keyboardController.enable(robot, model, data, parentContext.mujoco, {
        keyboardInputEnabled: developerMode
      });

      if (!developerMode) return;
      keyboardFolder = simulationFolder.addFolder("Keyboard Controls");
      // Add description labels - support multi-line descriptions
      const desc = keyboardController.getDescription();
      const lines = desc.split('\n');
      keyboardLabel = [];
      for (let i = 0; i < lines.length; i++) {
        const label = keyboardFolder.add({ info: lines[i] }, 'info').name('').disable();
        // Hide the label column to use full width for the value
        const labelDom = label.domElement.querySelector('.name');
        if (labelDom) labelDom.style.display = 'none';
        const valueDom = label.domElement.querySelector('.widget');
        if (valueDom) valueDom.style.width = '100%';
        keyboardLabel.push(label);
      }
      keyboardFolder.open();
    } else {
      keyboardController.disable();
    }
  };

  // Setup for initial scene
  await setupKeyboardControls(parentContext.model, parentContext.data, parentContext.params);

  // Update when scene changes (async callback)
  parentContext.updateGUICallbacks.push(async (model, data, params) => {
    await setupKeyboardControls(model, data, params);
  });

  // Add function that resets the camera to the default position.
  // Can be triggered by pressing ctrl + A.
  document.addEventListener('keydown', (event) => {
    if (event.ctrlKey && event.code === 'KeyA') {
      resetDefaultCamera(parentContext);
      event.preventDefault();
    }
  });
  actionInnerHTML += 'Reset free camera<br>';
  keyInnerHTML += 'Ctrl A<br>';

  parentContext.gui.open();
}

/** Frame Go2 from inside its spawn clearance so house walls do not hide the robot. */
export function resetDefaultCamera(parentContext) {
  const narrowViewport = window.matchMedia('(max-width: 640px)').matches;
  const preset = parentContext.sceneManager?.getCurrentCameraPreset();
  if (preset) {
    parentContext.camera.position.set(...preset.position);
    parentContext.controls.target.set(...preset.target);
    parentContext.controls.update();
    return;
  }
  const spawn = parentContext.sceneManager?.getCurrentSpawnPosition() ?? [0, 0];
  const targetX = spawn[0];
  const targetZ = -spawn[1];
  const indoors = parentContext.params.environment.startsWith('scenesmith_house_');
  if (indoors) {
    parentContext.camera.position.set(targetX - 0.7, 1.24, targetZ - 0.7);
    parentContext.controls.target.set(targetX, 0.25, targetZ);
    parentContext.controls.update();
    return;
  }
  parentContext.camera.position.set(
    targetX + 0.5,
    narrowViewport ? 2.2 : 1.7,
    targetZ + (narrowViewport ? -5.5 : -3)
  );
  parentContext.controls.target.set(targetX, 0.7, targetZ);
  parentContext.controls.update();
}


/** Release a rendered MuJoCo tree, including GPU resources owned by its meshes. */
function disposeMujocoRoot(parent) {
  const root = parent.mujocoRoot ?? parent.scene.getObjectByName('MuJoCo Root');
  if (!root) return;

  const geometries = new Set();
  const materials = new Set();
  const textures = new Set();
  root.traverse((object) => {
    if (object.geometry) geometries.add(object.geometry);
    const objectMaterials = Array.isArray(object.material)
      ? object.material
      : [object.material].filter(Boolean);
    for (const material of objectMaterials) {
      materials.add(material);
      for (const value of Object.values(material)) {
        if (value?.isTexture) textures.add(value);
      }
    }
    object.shadow?.map?.dispose?.();
    object.getRenderTarget?.().dispose?.();
  });

  parent.scene.remove(root);
  for (const texture of textures) texture.dispose();
  for (const material of materials) material.dispose();
  for (const geometry of geometries) geometry.dispose();
  parent.mujocoRoot = null;
  parent.bodies = {};
  parent.lights = [];
}


/** Loads a scene for MuJoCo
 * @param {mujoco} mujoco This is a reference to the mujoco namespace object
 * @param {string} filename This is the name of the .xml file in the /working/ directory of the MuJoCo/Emscripten Virtual File System
 * @param {MuJoCoDemo} parent The three.js Scene Object to add the MuJoCo model elements to
 */
export async function loadSceneFromURL(mujoco, filename, parent) {
    await keyboardController.disable();
    // Stop all consumers before releasing WASM-backed arrays, then free both
    // halves of the MuJoCo scene. MjModel is by far the largest allocation for
    // SceneSmith rooms and must not survive an environment switch.
    parent.sensorSuite?.configure(null, null, null);
    parent.dragStateManager?.configure(null, null);
    disposeMujocoRoot(parent);
    const previousData = parent.data;
    const previousModel = parent.model;
    parent.data = null;
    parent.model = null;
    previousData?.delete();
    previousModel?.delete();

    // Load in the state from XML.
    const xmlPath = "/working/"+filename;
    console.log('Loading MuJoCo model from:', xmlPath);

    // Debug: Check if the XML file exists and print its content
    try {
      const xmlContent = mujoco.FS.readFile(xmlPath, { encoding: 'utf8' });
      console.log('XML file content (first 500 chars):', xmlContent.substring(0, 500));
    } catch (e) {
      console.error('Failed to read XML file:', e);
    }

    try {
      parent.model = mujoco.MjModel.loadFromXML(xmlPath);
    } catch (e) {
      console.error('MuJoCo XML load error:', e);
      console.error('Error message:', e.message);
      // Try to get more info from MuJoCo's error handling
      if (mujoco.mjXError) {
        console.error('MuJoCo error details:', mujoco.mjXError);
      }
      throw e;
    }
    parent.data  = new mujoco.MjData(parent.model);

    let model = parent.model;
    let data = parent.data;

    // Decode the null-terminated string names.
    let textDecoder = new TextDecoder("utf-8");
    let names_array = new Uint8Array(model.names);
    let fullString = textDecoder.decode(model.names);
    let names = fullString.split(textDecoder.decode(new ArrayBuffer(1)));

    // Create the root object.
    let mujocoRoot = new THREE.Group();
    mujocoRoot.name = "MuJoCo Root";
    parent.scene.add(mujocoRoot);

    /** @type {Object.<number, THREE.Group>} */
    let bodies = {};
    /** @type {Object.<number, THREE.BufferGeometry>} */
    let meshes = {};
    /** @type {THREE.Light[]} */
    let lights = [];
    const materialCache = new Map();
    const textureCache = new Map();

    // Loop through the MuJoCo geoms and recreate them in three.js.
    for (let g = 0; g < model.ngeom; g++) {
      // Only visualize geom groups up to 2 (same default behavior as simulate).
      if (!(model.geom_group[g] < 3)) { continue; }

      // Get the body ID and type of the geom.
      let b    = model.geom_bodyid[g];
      let type = model.geom_type  [g];
      let size = [
        model.geom_size[(g*3) + 0],
        model.geom_size[(g*3) + 1],
        model.geom_size[(g*3) + 2]
      ];

      // Create the body if it doesn't exist.
      if (!(b in bodies)) {
        bodies[b] = new THREE.Group();
        
        let start_idx = model.name_bodyadr[b];
        let end_idx = start_idx;
        while (end_idx < names_array.length && names_array[end_idx] !== 0) {
          end_idx++;
        }
        let name_buffer = names_array.subarray(start_idx, end_idx);
        bodies[b].name = textDecoder.decode(name_buffer);
        
        bodies[b].bodyID = b;
        bodies[b].has_custom_mesh = false;
      }

      // Set the default geometry. In MuJoCo, this is a sphere.
      let geometry = new THREE.SphereGeometry(size[0] * 0.5);
      if (type == mujoco.mjtGeom.mjGEOM_PLANE.value) {
        // Special handling for plane later.
      } else if (type == mujoco.mjtGeom.mjGEOM_HFIELD.value) {
        // TODO: Implement this.
      } else if (type == mujoco.mjtGeom.mjGEOM_SPHERE.value) {
        geometry = new THREE.SphereGeometry(size[0]);
      } else if (type == mujoco.mjtGeom.mjGEOM_CAPSULE.value) {
        geometry = new THREE.CapsuleGeometry(size[0], size[1] * 2.0, 20, 20);
      } else if (type == mujoco.mjtGeom.mjGEOM_ELLIPSOID.value) {
        geometry = new THREE.SphereGeometry(1); // Stretch this below
      } else if (type == mujoco.mjtGeom.mjGEOM_CYLINDER.value) {
        geometry = new THREE.CylinderGeometry(size[0], size[0], size[1] * 2.0);
      } else if (type == mujoco.mjtGeom.mjGEOM_BOX.value) {
        geometry = new THREE.BoxGeometry(size[0] * 2.0, size[2] * 2.0, size[1] * 2.0);
      } else if (type == mujoco.mjtGeom.mjGEOM_MESH.value) {
        let meshID = model.geom_dataid[g];

        if (!(meshID in meshes)) {
          geometry = createMujocoMeshGeometry(model, meshID);
          meshes[meshID] = geometry;
        } else {
          geometry = meshes[meshID];
        }

        bodies[b].has_custom_mesh = true;
      }
      // Done with geometry creation.

      // Reuse decoded textures and materials across repeated furniture instances.
      const matId = model.geom_matid[g];
      let color = [
        model.geom_rgba[(g * 4) + 0],
        model.geom_rgba[(g * 4) + 1],
        model.geom_rgba[(g * 4) + 2],
        model.geom_rgba[(g * 4) + 3]];
      if (matId != -1) {
        color = [
          model.mat_rgba[(matId * 4) + 0],
          model.mat_rgba[(matId * 4) + 1],
          model.mat_rgba[(matId * 4) + 2],
          model.mat_rgba[(matId * 4) + 3]];
      }

      const materialKey = matId != -1 ? `material:${matId}` : `rgba:${color.join(',')}`;
      let currentMaterial = materialCache.get(materialKey);
      if (!currentMaterial) {
        let texture;

        // Construct Texture from model.tex_data
        // mat_texid is now a matrix (nmat x mjNTEXROLE)
        // We use mjTEXROLE_RGB (value 1) for standard diffuse/color textures
        const mjNTEXROLE = 10; // Total number of texture roles
        const mjTEXROLE_RGB = 1; // RGB texture role
        const texId = matId != -1
          ? model.mat_texid[(matId * mjNTEXROLE) + mjTEXROLE_RGB]
          : -1;

        if (texId != -1) {
          const repeatX = model.mat_texrepeat[(matId * 2) + 0];
          const repeatY = model.mat_texrepeat[(matId * 2) + 1];
          const textureKey = `${texId}:${repeatX}:${repeatY}`;
          texture = textureCache.get(textureKey);
          if (!texture) {
            texture = createMujocoColorTexture(model, texId, matId);
            textureCache.set(textureKey, texture);
          }
        }

        currentMaterial = new THREE.MeshToonMaterial({
          color: new THREE.Color(color[0], color[1], color[2]),
          transparent: color[3] < 1.0,
          opacity: color[3] < 1.0 ? color[3] : 1.0,
          map: texture
        });
        materialCache.set(materialKey, currentMaterial);
      }

      let mesh;// = new THREE.Mesh();
      if (type == 0) {
        mesh = new Reflector(new THREE.PlaneGeometry(100, 100), {
          clipBias: 0.003,
          texture: currentMaterial.map
        });
        mesh.rotateX( - Math.PI / 2 );
      } else {
        mesh = new THREE.Mesh(geometry, currentMaterial);
      }

      mesh.castShadow = g == 0 ? false : true;
      mesh.receiveShadow = type != 7;
      mesh.bodyID = b;
      bodies[b].add(mesh);
      getPosition  (model.geom_pos, g, mesh.position  );
      if (type != 0) { getQuaternion(model.geom_quat, g, mesh.quaternion); }
      if (type == 4) { mesh.scale.set(size[0], size[2], size[1]); } // Stretch the Ellipsoid
    }

    // Parse tendons.
    let tendonMat = new THREE.MeshPhongMaterial();
    tendonMat.color = new THREE.Color(0.8, 0.3, 0.3);
    mujocoRoot.cylinders = new THREE.InstancedMesh(
        new THREE.CylinderGeometry(1, 1, 1),
        tendonMat, 1023);
    mujocoRoot.cylinders.receiveShadow = true;
    mujocoRoot.cylinders.castShadow    = true;
    mujocoRoot.add(mujocoRoot.cylinders);
    mujocoRoot.spheres = new THREE.InstancedMesh(
        new THREE.SphereGeometry(1, 10, 10),
        tendonMat, 1023);
    mujocoRoot.spheres.receiveShadow = true;
    mujocoRoot.spheres.castShadow    = true;
    mujocoRoot.add(mujocoRoot.spheres);

    // Parse lights.
    for (let l = 0; l < model.nlight; l++) {
      let light = new THREE.DirectionalLight();
      if (model.light_type[l] == 0) {
        light = new THREE.SpotLight();
        light.angle = 1.51;//model.light_cutoffangle[l];
      } else if (model.light_type[l] == 1) {
        light = new THREE.DirectionalLight();
      } else if (model.light_type[l] == 2) {
        light = new THREE.PointLight();
      }else if (model.light_type[l] == 3) {
        light = new THREE.HemisphereLight();
      }

      light.angle = 1.11;

      light.decay = model.light_attenuation[l] * 100;
      light.penumbra = 0.5;
      light.castShadow = true; // default false
      light.intensity = light.intensity * 3.14 * 1.0;

      light.shadow.mapSize.width = 1024; // default
      light.shadow.mapSize.height = 1024; // default
      light.shadow.camera.near = 0.1; // default
      light.shadow.camera.far = 10; // default
      //bodies[model.light_bodyid()].add(light);
      if (bodies[0]) {
        bodies[0].add(light);
      } else {
        mujocoRoot.add(light);
      }
      lights.push(light);
    }
    if (model.nlight == 0) {
      let light = new THREE.DirectionalLight();
      mujocoRoot.add(light);
    }

    for (let b = 0; b < model.nbody; b++) {
      // Hidden collision-only bodies do not create a render mesh, but still need a
      // transform node so body updates and visual children remain well-defined.
      if (!bodies[b]) {
        bodies[b] = new THREE.Group();
        bodies[b].name = names[b + 1] ?? `body_${b}`;
        bodies[b].bodyID = b;
        bodies[b].has_custom_mesh = false;
      }
      if (b == 0) {
        mujocoRoot.add(bodies[b]);
      } else {
        bodies[0].add(bodies[b]);
      }
    }
  
    parent.mujocoRoot = mujocoRoot;

    return [model, data, bodies, lights];
}

export function drawTendonsAndFlex(mujocoRoot, model, data) {
  // Update tendon transforms.
  let identityQuat = new THREE.Quaternion();
  let numWraps = 0;
  if (mujocoRoot && mujocoRoot.cylinders) {
    let mat = new THREE.Matrix4();
    for (let t = 0; t < model.ntendon; t++) {
      let startW = data.ten_wrapadr[t];
      let r = model.tendon_width[t];
      for (let w = startW; w < startW + data.ten_wrapnum[t] -1 ; w++) {
        let tendonStart = getPosition(data.wrap_xpos, w    , new THREE.Vector3());
        let tendonEnd   = getPosition(data.wrap_xpos, w + 1, new THREE.Vector3());
        let tendonAvg   = new THREE.Vector3().addVectors(tendonStart, tendonEnd).multiplyScalar(0.5);

        let validStart = tendonStart.length() > 0.01;
        let validEnd   = tendonEnd  .length() > 0.01;

        if (validStart) { mujocoRoot.spheres.setMatrixAt(numWraps    , mat.compose(tendonStart, identityQuat, new THREE.Vector3(r, r, r))); }
        if (validEnd  ) { mujocoRoot.spheres.setMatrixAt(numWraps + 1, mat.compose(tendonEnd  , identityQuat, new THREE.Vector3(r, r, r))); }
        if (validStart && validEnd) {
          mat.compose(tendonAvg, identityQuat.setFromUnitVectors(
            new THREE.Vector3(0, 1, 0), tendonEnd.clone().sub(tendonStart).normalize()),
            new THREE.Vector3(r, tendonStart.distanceTo(tendonEnd), r));
          mujocoRoot.cylinders.setMatrixAt(numWraps, mat);
          numWraps++;
        }
      }
    }

    let curFlexSphereInd = numWraps;
    let tempvertPos = new THREE.Vector3();
    let tempvertRad = new THREE.Vector3();
    for (let i = 0; i < model.nflex; i++) {
      for(let j = 0; j < model.flex_vertnum[i]; j++) {
        let vertIndex = model.flex_vertadr[i] + j;
        getPosition(data.flexvert_xpos, vertIndex, tempvertPos);
        let r   = 0.01;
        mat.compose(tempvertPos, identityQuat, tempvertRad.set(r, r, r));

        mujocoRoot.spheres.setMatrixAt(curFlexSphereInd, mat);
        curFlexSphereInd++;
      }
    }
    mujocoRoot.cylinders.count = numWraps;
    mujocoRoot.spheres  .count = curFlexSphereInd;
    mujocoRoot.cylinders.instanceMatrix.needsUpdate = true;
    mujocoRoot.spheres  .instanceMatrix.needsUpdate = true;
  }
}

/** Access the vector at index, swizzle for three.js, and apply to the target THREE.Vector3
 * @param {Float32Array|Float64Array} buffer
 * @param {number} index
 * @param {THREE.Vector3} target */
export function getPosition(buffer, index, target, swizzle = true) {
  if (swizzle) {
    return target.set(
       buffer[(index * 3) + 0],
       buffer[(index * 3) + 2],
      -buffer[(index * 3) + 1]);
  } else {
    return target.set(
       buffer[(index * 3) + 0],
       buffer[(index * 3) + 1],
       buffer[(index * 3) + 2]);
  }
}

/** Access the quaternion at index, swizzle for three.js, and apply to the target THREE.Quaternion
 * @param {Float32Array|Float64Array} buffer
 * @param {number} index
 * @param {THREE.Quaternion} target */
export function getQuaternion(buffer, index, target, swizzle = true) {
  if (swizzle) {
    return target.set(
      -buffer[(index * 4) + 1],
      -buffer[(index * 4) + 3],
       buffer[(index * 4) + 2],
      -buffer[(index * 4) + 0]);
  } else {
    return target.set(
       buffer[(index * 4) + 0],
       buffer[(index * 4) + 1],
       buffer[(index * 4) + 2],
       buffer[(index * 4) + 3]);
  }
}

/** Converts this Vector3's Handedness to MuJoCo's Coordinate Handedness
 * @param {THREE.Vector3} target */
export function toMujocoPos(target) { return target.set(target.x, -target.z, target.y); }

/** Standard normal random number generator using Box-Muller transform */
export function standardNormal() {
  return Math.sqrt(-2.0 * Math.log( Math.random())) *
         Math.cos ( 2.0 * Math.PI * Math.random()); }
