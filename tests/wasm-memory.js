import assert from 'node:assert/strict';
import { readFile } from 'node:fs/promises';
import loadMujoco from 'mujoco-js';
import { Go2Controller } from '../assets/robots/go2/controller.js';
import { RobotSensorSuite } from '../src/utils/RobotSensorSuite.js';

/** Compare fresh WASM heaps under isolated control and sensor workloads using the real Go2 model. */
async function measure(scenario, steps) {
  const mujoco = await loadMujoco();
  const root = new URL('../assets/robots/go2/', import.meta.url);
  mujoco.FS.mkdir('/go2');
  mujoco.FS.mkdir('/go2/assets');
  for (const file of JSON.parse(await readFile(new URL('index.json', root)))) {
    mujoco.FS.writeFile(`/go2/${file}`, await readFile(new URL(file, root)));
  }
  mujoco.FS.writeFile('/go2/scene.xml', '<mujoco><include file="go2.xml"/><option timestep="0.002"/><worldbody><geom type="plane" size="0 0 .1" group="3"/><geom type="box" pos="3 0 1" size=".05 3 3" group="3"/></worldbody></mujoco>');
  const model = mujoco.MjModel.loadFromXML('/go2/scene.xml');
  const data = new mujoco.MjData(model);
  const controller = new Go2Controller(() => 0);
  await controller.initialize(model, data, mujoco);
  const sensors = new RobotSensorSuite(mujoco);
  sensors.configure(JSON.parse(await readFile(new URL('robot.json', root))), model, data);
  if (scenario === 'baseline' || scenario === 'options' || scenario === 'sensors') {
    controller.supportTorque = () => new Float64Array(model.nv);
  }
  const sampleSensors = scenario === 'sensors' || scenario === 'full';
  const samples = [];
  try {
    for (let step = 0; step <= steps; step++) {
      if (scenario === 'options') for (let i = 0; i < 10; i++) void model.opt.gravity;
      await controller.step({}, model, data, mujoco);
      mujoco.mj_step(model, data);
      if (sampleSensors && step % 50 === 0) { sensors.scanLidar(); sensors.scanPlanarLidar(); }
      if (sampleSensors && step % 100 === 0) sensors._captureDepth(sensors.cameras.get('front_rgbd'));
      if (step % 2500 === 0) {
        const sample = { scenario, step, heapBytes: data.qpos.buffer.byteLength, simulationTime: data.time };
        samples.push(sample);
        console.log(JSON.stringify(sample));
      }
      assert.ok(data.qpos.every(Number.isFinite));
    }
    const growth = samples.at(-1).heapBytes - samples[0].heapBytes;
    assert.ok(growth <= 16 * 1024 * 1024, `${scenario} WASM heap grew by ${growth} bytes`);
  } finally {
    sensors.dispose();
    await controller.dispose();
    data.delete();
    model.delete();
  }
}

for (const scenario of (process.env.WASM_SCENARIOS ?? 'baseline,options,support,sensors,full').split(',')) {
  await measure(scenario, Number(process.env.WASM_STEPS ?? 10000));
}
