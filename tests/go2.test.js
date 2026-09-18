import test from 'node:test';
import assert from 'node:assert/strict';
import { readFile } from 'node:fs/promises';
import loadMujoco from 'mujoco-js';
import { computeGo2Targets, Go2Controller, GO2_ACTUATORS } from '../assets/robots/go2/controller.js';
import { PolicyController } from '../src/policy/PolicyController.js';
import { generatePinholeDirections, rayRangeToOpticalDepth, RobotSensorSuite } from '../src/utils/RobotSensorSuite.js';
import { NativeBridgeClient } from '../src/ControlPanel.js';
import { MujocoBridgeClient } from '../src/utils/MujocoBridgeClient.js';
import { KeyboardController, keyboardController } from '../src/utils/KeyboardControl.js';

const config = JSON.parse(await readFile(new URL('../assets/robots/go2/policy/moe_rough/policy.json', import.meta.url)));

/** Build a small state with the actual policy joint order for inference/PD lifecycle tests. */
function fixture() {
  const model = { opt: { timestep: 0.002 }, actuator_ctrlrange: new Float64Array(24).map((_, i) => i % 2 ? 45 : -45) };
  const data = { time: 0, qpos: new Float64Array(19), qvel: new Float64Array(18), ctrl: new Float64Array(12) };
  data.qpos[3] = 1;
  data.qpos.set(config.default_joint_pos, 7);
  const joints = new Map(GO2_ACTUATORS.map((name, i) => [name, { qpos: i + 7, qvel: i + 6 }]));
  const actuators = new Map(GO2_ACTUATORS.map((name, i) => [name, i]));
  return { model, data, joints, actuators };
}

/** Emulate only the ONNX boundary; preserve real observation, timing, validation and PD code. */
function runtime(run, release = async () => {}) {
  return {
    env: { wasm: {} }, Tensor: class { constructor(type, data, dims) { Object.assign(this, { type, data, dims }); } },
    InferenceSession: { create: async () => ({ inputNames: ['history'], outputNames: ['actions', 'weights', 'latent'], run, release }) }
  };
}

/** Serve the checked-in config and dummy weights to an injected inference session. */
function mockFetch(t) {
  t.mock.method(globalThis, 'fetch', async (url) => String(url).endsWith('.json')
    ? new Response(JSON.stringify(config)) : new Response(new Uint8Array([1])));
}

/** Verify continuous Twist remains fractional and invalid messages cannot overwrite it. */
test('Go2 accepts continuous Twist and clears it on disconnect', () => {
  const controller = new Go2Controller();
  assert.equal(controller.policy.status.loaded, false);
  assert.equal(controller.setTwist({ linearX: 0.123, linearY: -0.042, angularZ: 0.17 }), true);
  assert.deepEqual(controller.twist, { linearX: 0.123, linearY: -0.042, angularZ: 0.17 });
  assert.equal(controller.setTwist({ linearX: NaN }), false);
  assert.equal(controller.twist.linearX, 0.123);
  controller.setExternalControlEnabled(false);
  assert.deepEqual(controller.twist, { linearX: 0, linearY: 0, angularZ: 0 });
});

/** A pure yaw target uses tangential foot travel instead of translating the base. */
test('scripted yaw targets include front-rear lateral travel', () => {
  const targets = computeGo2Targets({ forward: 0, lateral: 0, turn: 0.24 }, Math.PI * 0.2);
  assert.ok(['FL', 'FR', 'RL', 'RR'].every((leg) => Math.abs(targets[`${leg}_hip`]) > 1e-4));
});

/** Policy commands compensate measured velocity while preserving a hard zero command. */
test('Go2 policy velocity tracking boosts low commands and clears compensation on stop', () => {
  const controller = new Go2Controller();
  controller.baseBody = 0;
  const model = { opt: { timestep: 0.02 } };
  const data = { xmat: new Float64Array([1, 0, 0, 0, 1, 0, 0, 0, 1]), qvel: new Float64Array(6) };
  const forward = controller.trackedPolicyCommand({ linearX: 0.18, linearY: 0, angularZ: 0 }, model, data);
  assert.ok(forward.linearX > 0.36 && forward.linearX < 0.38);
  const turn = controller.trackedPolicyCommand({ linearX: 0, linearY: 0, angularZ: -0.3 }, model, data);
  assert.ok(turn.angularZ < -0.68 && turn.angularZ > -0.72);
  assert.deepEqual(controller.trackedPolicyCommand(
    { linearX: 0, linearY: 0, angularZ: 0 }, model, data),
  { linearX: 0, linearY: 0, angularZ: 0 });
  assert.ok(controller.policyVelocityIntegral.every((value) => value === 0));
});

/** The Web watchdog spans slow rendered frames while disconnect still stops immediately. */
test('policy command expires after two seconds without a fresh Twist', async () => {
  let time = 0;
  const controller = new Go2Controller(() => time);
  controller.initialized = true;
  const commands = [];
  controller.policy.step = async (_model, _data, _joints, _actuators, command) => { commands.push({ ...command }); return true; };
  controller.setTwist({ linearX: 0.2 });
  time = 0.3;
  await controller.step({}, {}, {}, {});
  assert.equal(commands.at(-1).linearX, 0.2);
  controller.setTwist({ linearX: 0.15 });
  time = 0.6;
  await controller.step({}, {}, {}, {});
  assert.equal(commands.at(-1).linearX, 0.15);
  time = 2.601;
  await controller.step({}, {}, {}, {});
  assert.equal(commands.at(-1).linearX, 0);
});

/** Held manual keys override even a connected bridge, while stale external commands cannot reset manual gait. */
test('manual input overrides external Twist and remains active beyond the external watchdog', async () => {
  let time = 0;
  const controller = new Go2Controller(() => time);
  controller.initialized = true;
  const commands = [];
  controller.policy.step = async (_m, _d, _j, _a, command) => { commands.push({ ...command }); return true; };
  controller.setTwist({ linearX: -0.2 });
  time = 1;
  controller.gaitBlend = 0.8;
  await controller.step({ KeyW: true }, {}, {}, {});
  assert.equal(commands.at(-1).linearX, 0.6);
  assert.equal(controller.gaitBlend, 0.8);
  await controller.step({ KeyW: true, Space: true }, {}, {}, {});
  assert.deepEqual(commands.at(-1), { linearX: 0, linearY: 0, angularZ: 0 });
});

/** Release and blur stop stored Twist; typing and selection never activate robot motion. */
test('keyboard handlers stop on release/blur and ignore all editable targets', () => {
  const keyboard = new KeyboardController();
  keyboard.enabled = true;
  keyboard.keyStates = { KeyW: false, Space: false };
  keyboard.customController = new Go2Controller();
  const event = { code: 'KeyW', target: { tagName: 'BODY' }, preventDefault() {} };
  for (const target of [{ tagName: 'INPUT' }, { tagName: 'TEXTAREA' }, { tagName: 'SELECT' }, { tagName: 'DIV', isContentEditable: true }]) {
    keyboard._onKeyDown({ ...event, target });
    assert.equal(keyboard.keyStates.KeyW, false);
  }
  keyboard.customController.setTwist({ linearX: -0.2 });
  keyboard._onKeyDown(event);
  assert.equal(keyboard.keyStates.KeyW, true);
  keyboard._onKeyUp(event);
  assert.equal(keyboard.keyStates.KeyW, false);
  assert.equal(keyboard.customController.twist.linearX, 0);
  keyboard._onKeyDown(event);
  keyboard.customController.setTwist({ linearX: -0.2 });
  keyboard._onBlur();
  assert.equal(keyboard.keyStates.KeyW, false);
  assert.equal(keyboard.customController.twist.linearX, 0);
});

/** Contact-vector and entry copies must be released once per support step, including exceptional reads. */
test('support torque releases every owned Embind contact copy', () => {
  const controller = new Go2Controller();
  controller.feet = [];
  controller.supportBuffer = { GetView: () => new Float64Array(18) };
  for (const fail of [false, true]) {
    let vectorReads = 0;
    let vectorDeletes = 0;
    let contactDeletes = 0;
    const contact = {
      get frame() { if (fail) throw new Error('contact read failed'); return [0, 0, 1]; },
      dist: 0, geom: [1, 2], delete: () => contactDeletes++
    };
    const data = {
      get contact() {
        vectorReads++;
        return { size: () => 4, get: () => contact, delete: () => vectorDeletes++ };
      }
    };
    if (fail) assert.throws(() => controller.supportTorque({}, data), /contact read failed/);
    else controller.supportTorque({}, data);
    assert.equal(vectorReads, 1);
    assert.equal(vectorDeletes, 1);
    assert.equal(contactDeletes, fail ? 1 : 4);
  }
});

/** Check the upstream five-frame ordering, scaled velocity observations, 50 Hz cadence and 20/.5 PD. */
test('policy preserves 5x45 history, 50 Hz, gains 20/.5 and action scale .25', async (t) => {
  mockFetch(t);
  const frames = [];
  let releases = 0;
  const policy = new PolicyController(async () => runtime(async (inputs) => {
    frames.push(inputs.history);
    return { actions: { data: new Float32Array(12).fill(1) } };
  }, async () => { releases++; }));
  const { model, data, joints, actuators } = fixture();
  const before = data.qpos.slice();
  await policy.load(model);
  assert.deepEqual(data.qpos, before);
  data.qvel[6] = 2;
  for (let i = 0; i < 25; i++) {
    data.time = i * 0.002;
    await policy.step(model, data, joints, actuators, { linearX: 0.123, linearY: -0.1, angularZ: 0.2 });
  }
  assert.equal(frames.length, 3);
  assert.deepEqual(frames[0].dims, [1, 225]);
  assert.ok(frames[0].data.slice(0, 180).every((value) => value === 0));
  assert.ok(Math.abs(frames[0].data[186] - 0.246) < 1e-6);
  assert.ok(Math.abs(frames[0].data[187] + 0.2) < 1e-6);
  assert.ok(Math.abs(frames[0].data[188] - 0.05) < 1e-6);
  assert.equal(frames[1].data[213], 1);
  assert.ok(Math.abs(data.ctrl[0] - 4) < 1e-5);
  await policy.unload();
  assert.equal(releases, 1);
  assert.equal(policy.status.loaded, false);
});

/** A failed inference must release its runtime, show the error and return control to scripted gait. */
test('policy exposes inference errors and releases the failed session', async (t) => {
  mockFetch(t);
  let released = false;
  const policy = new PolicyController(async () => runtime(async () => { throw new Error('inference exploded'); }, async () => { released = true; }));
  const { model, data, joints, actuators } = fixture();
  await policy.load(model);
  assert.equal(await policy.step(model, data, joints, actuators, { linearX: 0, linearY: 0, angularZ: 0 }), false);
  assert.equal(released, true);
  assert.equal(policy.status.loaded, false);
  assert.match(policy.status.error, /inference exploded/);
});

/** Unloading during a pending download must never reactivate a cancelled session. */
test('unload cancels an in-flight explicit load', async (t) => {
  mockFetch(t);
  let resolve;
  let released = false;
  const ort = runtime(async () => ({}), async () => { released = true; });
  const session = await ort.InferenceSession.create();
  ort.InferenceSession.create = () => new Promise((done) => { resolve = done; });
  const policy = new PolicyController(async () => ort);
  const load = policy.load(fixture().model);
  while (!resolve) await new Promise((done) => setImmediate(done));
  await policy.unload();
  resolve(session);
  assert.equal(await load, false);
  assert.equal(policy.status.loaded, false);
  assert.equal(released, true);
});

/** Reset must discard a pre-reset ONNX result even if inference finishes after the reset. */
test('reset invalidates an outstanding inference action', async (t) => {
  mockFetch(t);
  let finish;
  const policy = new PolicyController(async () => runtime(() => new Promise((resolve) => { finish = resolve; })));
  const { model, data, joints, actuators } = fixture();
  await policy.load(model);
  const step = policy.step(model, data, joints, actuators, { linearX: 0.1, linearY: 0, angularZ: 0 });
  policy.reset();
  finish({ actions: { data: new Float32Array(12).fill(10) } });
  assert.equal(await step, false);
  assert.equal(policy.targets, null);
  assert.ok(policy.history.toArray().every((value) => value === 0));
  assert.ok(data.ctrl.every((value) => value === 0));
  await policy.unload();
});

/** A flat optical plane must have constant Z depth across the entire pinhole image. */
test('RGB-D depth is optical Z rather than increasing radial range', () => {
  const directions = generatePinholeDirections(160, 120, 60);
  for (let i = 2; i < directions.length; i += 3) {
    const range = 3 / -directions[i];
    assert.ok(Math.abs(rayRangeToOpticalDepth(range, directions[i]) - 3) < 1e-12);
  }
  assert.equal(rayRangeToOpticalDepth(-1, -1), Infinity);
});

/** Native policy buttons must wait for HTTP acknowledgements and expose runtime rejection. */
test('native panel uses shared HTTP commands and state', async () => {
  const calls = [];
  const client = new NativeBridgeClient({ search: '', protocol: 'http:', hostname: 'localhost' }, async (url, options) => {
    calls.push({ url, options });
    if (url.endsWith('/command')) return new Response(JSON.stringify({ accepted: false, error: 'missing weights' }));
    return new Response(JSON.stringify({ policy: { id: 'moe_rough', loaded: false, error: null } }));
  });
  await assert.rejects(client.command({ type: 'policy', action: 'load', id: 'moe_rough' }), /missing weights/);
  assert.equal(calls[0].url, 'http://localhost:8766/command');
  assert.deepEqual(JSON.parse(calls[0].options.body), { type: 'policy', action: 'load', id: 'moe_rough' });
  await client.refresh();
  assert.equal(client.state.policy.loaded, false);
});

/** Bridge policy acknowledgements must follow the completed asynchronous action and include failure text. */
test('web bridge waits for policy acknowledgement and publishes top-level policy status', async (t) => {
  const bridge = Object.create(MujocoBridgeClient.prototype);
  bridge.status = { stateFrames: 0 };
  bridge.demo = { params: { environment: 'go2_rl_stairs' }, data: { time: 1 }, sensorSuite: { latestImu: null } };
  const messages = [];
  bridge._send = (message) => messages.push(message);
  let finish;
  t.mock.method(keyboardController, 'policyCommand', () => new Promise((resolve) => { finish = resolve; }));
  t.mock.method(keyboardController, 'getRobotState', () => ({ joints: { names: [] } }));
  t.mock.method(keyboardController, 'getPolicyStatus', () => ({ id: 'moe_rough', loaded: true, error: null }));
  const handling = bridge._handleMessage(JSON.stringify({ type: 'policy', action: 'load', id: 'moe_rough', sequence: 42 }));
  assert.equal(messages.length, 0);
  finish(true);
  await handling;
  assert.deepEqual(messages[0], { type: 'command_ack', sequence: 42, accepted: true, error: null });
  assert.equal(messages[1].policy.loaded, true);
  t.mock.method(keyboardController, 'policyCommand', async () => { throw new Error('weights unavailable'); });
  await bridge._handleMessage(JSON.stringify({ type: 'policy', action: 'load', id: 'moe_rough', sequence: 43 }));
  assert.equal(messages[2].accepted, false);
  assert.equal(messages[2].error, 'weights unavailable');
});

/** Sensor backpressure bounds the WebSocket queue while command acknowledgements bypass it. */
test('web bridge drops bulk frames before they can delay command acknowledgements', (t) => {
  const bridge = Object.create(MujocoBridgeClient.prototype);
  const sent = [];
  bridge.socket = { readyState: 1, bufferedAmount: 3 * 1024 * 1024, send: (value) => sent.push(value) };
  t.mock.method(globalThis, 'WebSocket', class { static OPEN = 1; });
  assert.equal(bridge._send({ type: 'camera', data: 'bulk' }), false);
  assert.equal(bridge._send({ type: 'command_ack', sequence: 7, accepted: true }, true), true);
  assert.deepEqual(JSON.parse(sent[0]), { type: 'command_ack', sequence: 7, accepted: true });
});

/** Load the real Go2 mesh model into MuJoCo WASM and exercise motor-driven locomotion. */
test('actual WASM Go2 physics stands and responds to continuous Twist', { timeout: 120000 }, async () => {
  const mujoco = await loadMujoco();
  mujoco.FS.mkdir('/go2');
  mujoco.FS.mkdir('/go2/assets');
  const root = new URL('../assets/robots/go2/', import.meta.url);
  const files = JSON.parse(await readFile(new URL('index.json', root)));
  for (const file of files) mujoco.FS.writeFile(`/go2/${file}`, await readFile(new URL(file, root)));
  mujoco.FS.writeFile('/go2/scene.xml', '<mujoco><include file="go2.xml"/><option timestep="0.002"/><worldbody><geom type="plane" size="0 0 .1" group="3"/><geom type="box" pos="3 0 1" size=".05 3 3" group="3"/></worldbody></mujoco>');
  const model = mujoco.MjModel.loadFromXML('/go2/scene.xml');
  const data = new mujoco.MjData(model);
  let wallTime = 0;
  const controller = new Go2Controller(() => wallTime);
  await controller.initialize(model, data, mujoco);
  const robot = JSON.parse(await readFile(new URL('robot.json', root)));
  const suite = new RobotSensorSuite(mujoco);
  suite.configure(robot, model, data);
  const camera = suite.cameras.get('front_rgbd');
  const depth = suite._captureDepth(camera);
  for (const index of [0, 79, 159, 30 * 160, 30 * 160 + 159]) {
    assert.ok(Math.abs(depth.data[index] - 2.63) < 1e-4, `flat wall depth ${depth.data[index]}`);
  }
  assert.ok(Array.from(depth.geomIds).every((id) => id < 0 || model.geom_group[id] === 3));
  suite.configure(null, null, null);
  for (let i = 0; i < 500; i++) { await controller.step({}, model, data, mujoco); mujoco.mj_step(model, data); }
  const start = data.qpos.slice(0, 3);
  assert.ok(start[2] > 0.2, `standing height ${start[2]}`);
  controller.setTwist({ linearX: 0.3, linearY: 0, angularZ: 0 });
  for (let i = 0; i < 1500; i++) {
    wallTime = i * 0.002;
    if (i % 50 === 0) controller.setTwist({ linearX: 0.3 });
    await controller.step({}, model, data, mujoco);
    mujoco.mj_step(model, data);
  }
  assert.ok(data.qpos.every(Number.isFinite));
  assert.ok(data.qpos[0] > start[0] + 0.05, `forward displacement ${data.qpos[0] - start[0]}`);
  assert.ok(data.qpos[2] > 0.2, `moving height ${data.qpos[2]}`);
  assert.equal(controller.getRobotState(model, data).joints.names.length, 12);
  wallTime += 2.001;
  await controller.step({}, model, data, mujoco);
  assert.deepEqual(controller.twist, { linearX: 0, linearY: 0, angularZ: 0 });

  controller.reset(model, data);
  for (let i = 0; i < 500; i++) {
    wallTime += 0.002;
    await controller.step({}, model, data, mujoco);
    mujoco.mj_step(model, data);
  }
  const startYaw = Math.atan2(
    2 * (data.qpos[3] * data.qpos[6] + data.qpos[4] * data.qpos[5]),
    1 - 2 * (data.qpos[5] ** 2 + data.qpos[6] ** 2));
  const turnStart = data.qpos.slice(0, 2);
  for (let i = 0; i < 1500; i++) {
    wallTime += 0.002;
    if (i % 50 === 0) controller.setTwist({ angularZ: 0.12 });
    await controller.step({}, model, data, mujoco);
    mujoco.mj_step(model, data);
  }
  const endYaw = Math.atan2(
    2 * (data.qpos[3] * data.qpos[6] + data.qpos[4] * data.qpos[5]),
    1 - 2 * (data.qpos[5] ** 2 + data.qpos[6] ** 2));
  const yawDelta = Math.atan2(Math.sin(endYaw - startYaw), Math.cos(endYaw - startYaw));
  assert.ok(yawDelta > 0.08, `low-speed yaw displacement ${yawDelta}`);
  assert.ok(Math.hypot(data.qpos[0] - turnStart[0], data.qpos[1] - turnStart[1]) < 0.16,
    `in-place turn translation ${Math.hypot(data.qpos[0] - turnStart[0], data.qpos[1] - turnStart[1])}`);
  assert.ok(data.qpos[2] > 0.2, `turning height ${data.qpos[2]}`);
  await controller.dispose();
  data.delete();
  model.delete();
});
