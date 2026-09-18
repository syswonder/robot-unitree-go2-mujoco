import { chromium } from 'playwright';
import assert from 'node:assert/strict';

const base = process.env.WEB_TEST_URL ?? 'http://127.0.0.1:5182';
const nativeBase = process.env.NATIVE_TEST_URL ?? 'http://127.0.0.1:5181';
const browser = await chromium.launch({ headless: true, args: ['--no-sandbox', '--use-gl=angle', '--use-angle=swiftshader'] });
const errors = [];

/** Assert the visible WebGL canvas contains rendered geometry, not a uniform background. */
async function checkCanvas(page) {
  const result = await page.evaluate(() => {
    const canvas = document.querySelector('#mujoco-canvas');
    const gl = canvas.getContext('webgl2');
    const pixels = new Uint8Array(gl.drawingBufferWidth * gl.drawingBufferHeight * 4);
    gl.readPixels(0, 0, gl.drawingBufferWidth, gl.drawingBufferHeight, gl.RGBA, gl.UNSIGNED_BYTE, pixels);
    const colors = new Set();
    for (let i = 0; i < pixels.length; i += 400) colors.add(`${pixels[i]},${pixels[i + 1]},${pixels[i + 2]}`);
    return { colors: colors.size, width: canvas.width, height: canvas.height };
  });
  assert.ok(result.colors > 20, JSON.stringify(result));
  return result;
}

try {
  if (process.env.WEB_ONLY !== '1') {
  const page = await browser.newPage({ viewport: { width: 1280, height: 800 } });
  page.on('pageerror', (error) => errors.push(error.message));
  const nativeRequests = [];
  page.on('request', (request) => nativeRequests.push(request.url()));
  await page.goto(`${nativeBase}/?backend=native`);
  await page.waitForFunction(() => !document.querySelector('#policy-load').disabled);
  assert.equal(await page.locator('canvas').count(), 0);
  assert.ok(!nativeRequests.some((url) => /mujoco_wasm|policy\.onnx|ort\.wasm/.test(url)));
  await page.locator('#policy-load').click();
  await page.waitForFunction(() => document.querySelector('#policy-status').textContent === 'MoE Rough loaded', null, { timeout: 120000 });
  await page.locator('#policy-unload').click();
  await page.waitForFunction(() => document.querySelector('#policy-status').textContent.includes('ONNX unloaded'));
  assert.equal((await (await page.request.get('http://127.0.0.1:8766/state')).json()).policy.loaded, false);
  await page.screenshot({ path: '/tmp/go2-native-panel.png' });
  await page.goto(`${nativeBase}/`);
  await page.waitForFunction(() => !document.querySelector('#policy-load').disabled);
  assert.equal(await page.locator('canvas').count(), 0, 'ordinary visits must not start another simulator');
  assert.ok(!nativeRequests.some((url) => /mujoco_wasm|policy\.onnx|ort\.wasm/.test(url)));
  console.log('Native load/unload passed; no browser physics or motion commands; left unloaded.');
  await page.close();
  }

  if (process.env.NATIVE_ONLY !== '1') {
  const web = await browser.newPage({ viewport: { width: 1280, height: 800 } });
  web.on('pageerror', (error) => errors.push(error.message));
  const requests = [];
  web.on('request', (request) => requests.push(request.url()));
  await web.goto(`${base}/?bridge=off&environment=go2_rl_stairs`);
  await web.waitForFunction(() => window.mujocoApp?.getState().simulationTime > 0.6, null, { timeout: 120000 });
  assert.ok(!requests.some((url) => /policy\.onnx|onnxruntime/.test(url)), 'policy must not preload');
  const before = await web.evaluate(() => window.mujocoApp.getState());
  assert.equal(before.policy.loaded, false);
  assert.equal(before.robot, 'go2');
  assert.ok(before.interaction.orbitEnabled);
  console.log('Web canvas', await checkCanvas(web));
  await web.mouse.move(850, 450);
  await web.mouse.down();
  await web.mouse.move(920, 490, { steps: 10 });
  await web.mouse.up();
  const orbit = await web.evaluate(() => window.mujocoApp.getState().camera);
  assert.notDeepEqual(orbit.position, before.camera.position);
  assert.equal(await web.locator('#control-panel').count(), 0);
  await web.locator('#policy-folder > .title').click();
  assert.equal(await web.locator('#policy-strategy select').inputValue(), 'MoE Rough Terrain');
  assert.equal(await web.evaluate(() => document.querySelector('#policy-strategy select').selectedOptions[0].textContent), 'MoE Rough Terrain');
  await web.locator('#policy-toggle button').click();
  await web.waitForFunction(() => window.mujocoApp.getState().policy.loaded || window.mujocoApp.getState().policy.error, null, { timeout: 120000 });
  assert.equal((await web.evaluate(() => window.mujocoApp.getState().policy)).error, null);
  assert.equal(await web.evaluate(() => window.mujocoApp.getState().policy.loaded), true);
  const loadedTime = await web.evaluate(() => window.mujocoApp.getState().simulationTime);
  await web.waitForFunction((time) => window.mujocoApp.getState().simulationTime > time + 0.5, loadedTime);
  const sensors = await web.evaluate(() => {
    const frame = window.mujocoSensors.captureCamera('front_rgbd');
    const lidar = window.mujocoSensors.scanLidar();
    const imu = window.mujocoSensors.readImu();
    return { width: frame.width, height: frame.height, depth: [frame.depth.width, frame.depth.height],
      finiteDepths: Array.from(frame.depth.data).filter(Number.isFinite).length,
      lidarPoints: lidar.pointsLocal.length, imu, state: window.mujocoBridge.robotState() };
  });
  assert.deepEqual([sensors.width, sensors.height, ...sensors.depth], [320, 240, 160, 120]);
  assert.ok(sensors.finiteDepths > 0);
  assert.ok(sensors.lidarPoints > 0);
  assert.equal(sensors.state.joints.names.length, 12);
  await web.screenshot({ path: '/tmp/go2-web-desktop.png' });
  await web.locator('#policy-toggle button').click();
  await web.waitForFunction(() => !window.mujocoApp.getState().policy.loaded);
  await web.setViewportSize({ width: 390, height: 844 });
  await web.waitForTimeout(300);
  console.log('Mobile canvas', await checkCanvas(web));
  const overflow = await web.locator('.lil-gui.root').evaluate((element) => ({ width: element.scrollWidth, client: element.clientWidth, rect: element.getBoundingClientRect().toJSON() }));
  assert.ok(overflow.width <= overflow.client);
  assert.ok(overflow.rect.left >= 0 && overflow.rect.right <= 390);
  await web.screenshot({ path: '/tmp/go2-web-mobile.png' });
  await web.close();
  for (const environment of ['scenesmith_house_185', 'scenesmith_house_186', 'go2_rl_track']) {
    const scene = await browser.newPage({ viewport: { width: 1280, height: 800 } });
    scene.on('pageerror', (error) => errors.push(error.message));
    await scene.goto(`${base}/?bridge=off&environment=${environment}`);
    await scene.waitForFunction(() => window.mujocoApp?.getState().simulationTime > 0.2 || document.querySelector('#startup-error'), null, { timeout: 90000 });
    assert.equal(await scene.locator('#startup-error').count(), 0);
    assert.equal(await scene.evaluate(() => window.mujocoApp.getState().robot), 'go2');
    await checkCanvas(scene);
    await scene.screenshot({ path: `/tmp/go2-${environment}.png` });
    if (environment.startsWith('scenesmith')) {
      await scene.setViewportSize({ width: 390, height: 844 });
      await scene.waitForTimeout(300);
      await checkCanvas(scene);
      await scene.screenshot({ path: `/tmp/go2-${environment}-mobile.png` });
    }
    await scene.close();
  }
  assert.deepEqual(errors, []);
  console.log('Web policy load/infer/unload, sensors, orbit, desktop and mobile passed.');
  }
} finally {
  await browser.close();
}
