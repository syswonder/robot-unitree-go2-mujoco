import assert from 'node:assert/strict';
import { chromium } from 'playwright';

const browser = await chromium.launch({ headless: true, args: ['--no-sandbox'] });
try {
  const page = await browser.newPage();
  await page.goto(`${process.env.WEB_TEST_URL ?? 'http://127.0.0.1:5182'}/?backend=native`);
  const result = await page.evaluate(async () => {
    const { SceneManager } = await import('/src/utils/SceneManager.js');
    let xml = '<mujoco><worldbody><body name="base_link" pos="1 2 .4" quat=".707106781 0 .707106781 0"><freejoint/></body></worldbody><keyframe><key name="home" qpos="1 2 .4 .707106781 0 .707106781 0 .9 -1.8"/></keyframe></mujoco>';
    const manager = {
      mujoco: { FS: { readFile: () => xml } },
      _writeToFS: (_path, value) => { xml = value; }
    };
    SceneManager.prototype._applyRobotSpawn.call(manager, '/go2.xml', { position: [4, 5], yaw: Math.PI }, 'go2');
    const document = new DOMParser().parseFromString(xml, 'text/xml');
    const body = document.querySelector('body');
    return {
      position: body.getAttribute('pos').split(' ').map(Number),
      quaternion: body.getAttribute('quat').split(' ').map(Number),
      qpos: document.querySelector('key').getAttribute('qpos').split(' ').map(Number)
    };
  });
  assert.deepEqual(result.position, [3, 3, 0.4]);
  assert.deepEqual(result.qpos.slice(0, 3), result.position);
  assert.deepEqual(result.qpos.slice(3, 7), result.quaternion);
  assert.ok(Math.abs(result.quaternion[1] + Math.SQRT1_2) < 1e-8);
  assert.ok(Math.abs(result.quaternion[3] - Math.SQRT1_2) < 1e-8);
  assert.deepEqual(result.qpos.slice(7), [0.9, -1.8]);
  console.log('PASS: body and home keyframe positions/quaternions use the same world yaw transform');
} finally {
  await browser.close();
}
