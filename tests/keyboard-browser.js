import assert from 'node:assert/strict';
import { once } from 'node:events';
import { chromium } from 'playwright';
import bundle from 'playwright-core/lib/utilsBundle';

// An isolated protocol observer exercises the real WebSocket client without replacing the ROS runtime.
const bridge = new bundle.wsServer({ host: '127.0.0.1', port: 0 });
await once(bridge, 'listening');
const frames = {};
let socket;
bridge.on('connection', (client) => {
  socket = client;
  client.on('message', (raw) => {
    const message = JSON.parse(raw);
    frames[message.type] = (frames[message.type] ?? 0) + 1;
  });
});
const browser = await chromium.launch({ headless: true, args: ['--no-sandbox'] });
const errors = [];
let page;
try {
  page = await browser.newPage({ viewport: { width: 1280, height: 800 } });
  page.on('pageerror', (error) => errors.push(error.message));
  page.on('console', (message) => { if (message.type() === 'error') console.log(message.text()); });
  await page.goto(`${process.env.WEB_TEST_URL ?? 'http://127.0.0.1:5182'}/?runtime=1&environment=go2_rl_stairs&bridgeUrl=ws://127.0.0.1:${bridge.address().port}`);
  await page.waitForFunction(() => window.mujocoBridge?.status().connected && window.mujocoApp.getState().simulationTime > 0.8, null, { timeout: 90000 });
  assert.ok(!new URL(page.url()).searchParams.has('dev'));
  socket.send(JSON.stringify({ type: 'cmd_vel', linearX: -0.2, linearY: 0, angularZ: 0, sequence: 10 }));
  await page.waitForTimeout(50);
  await page.keyboard.down('w');
  await page.keyboard.down('Space');
  const production = await page.evaluate(async () => {
    const { keyboardController } = await import('/src/utils/KeyboardControl.js');
    return {
      keyboardInputEnabled: keyboardController.keyboardInputEnabled,
      keys: keyboardController.keyStates,
      twist: keyboardController.customController.twist,
      paused: window.mujocoApp.getState().paused
    };
  });
  await page.keyboard.up('Space');
  await page.keyboard.up('w');
  assert.equal(production.keyboardInputEnabled, false);
  assert.ok(Object.values(production.keys).every((value) => !value));
  assert.deepEqual(production.twist, { linearX: -0.2, linearY: 0, angularZ: 0 });
  assert.equal(production.paused, false);
  await page.locator('#policy-folder > .title').click();
  await page.locator('#policy-toggle button').click();
  await page.waitForFunction(() => window.mujocoApp.getState().policy.loaded, null, { timeout: 120000 });
  await page.locator('#policy-toggle button').click();
  await page.waitForFunction(() => !window.mujocoApp.getState().policy.loaded);
  for (let attempt = 0; attempt < 50 &&
       !(frames.state > 0 && frames.camera > 0 && frames.scan > 0 && frames.command_ack > 0); attempt++) {
    await page.waitForTimeout(100);
  }
  assert.ok(
    frames.state > 0 && frames.camera > 0 && frames.scan > 0 && frames.command_ack > 0,
    JSON.stringify(frames)
  );
  assert.deepEqual(errors, []);
  await page.close();

  page = await browser.newPage({ viewport: { width: 1280, height: 800 } });
  await page.goto(`${process.env.WEB_TEST_URL ?? 'http://127.0.0.1:5182'}/?runtime=1&dev=1&environment=go2_rl_stairs&bridgeUrl=ws://127.0.0.1:${bridge.address().port}`);
  await page.waitForFunction(() => window.mujocoBridge?.status().connected && window.mujocoApp.getState().simulationTime > 0.4, null, { timeout: 90000 });
  await page.keyboard.down('w');
  const development = await page.evaluate(async () => {
    const { keyboardController } = await import('/src/utils/KeyboardControl.js');
    return { keyboardInputEnabled: keyboardController.keyboardInputEnabled, keyW: keyboardController.keyStates.KeyW };
  });
  await page.keyboard.up('w');
  assert.deepEqual(development, { keyboardInputEnabled: true, keyW: true });
  console.log(JSON.stringify({ observerFrames: frames, productionKeyboardDisabled: true, developmentKeyboardEnabled: true }));
} catch (error) {
  console.log(await page?.evaluate(() => ({ state: window.mujocoApp?.getState(), error: document.querySelector('#startup-error')?.textContent })));
  throw error;
} finally {
  await browser.close();
  for (const client of bridge.clients) client.terminate();
  await new Promise((resolve) => bridge.close(resolve));
}
