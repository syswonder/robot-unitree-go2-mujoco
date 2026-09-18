import assert from 'node:assert/strict';
import { readFile, writeFile } from 'node:fs/promises';

const bridgeUrl = process.env.BRIDGE_HTTP_URL ?? 'http://127.0.0.1:8766';
const durationSeconds = Number(process.env.SOAK_SECONDS ?? 300);
const logPath = new URL('../.runtime/browser.log', import.meta.url);
const samples = [];

/** Read the actual ROS bridge with a timeout so stale or stopped runtimes fail the soak test. */
async function readBridge(path) {
  const response = await fetch(`${bridgeUrl}${path}`, { signal: AbortSignal.timeout(4000) });
  assert.ok(response.ok, `${path}: HTTP ${response.status}`);
  return response.json();
}

/** Wait for the coordinated fresh web boot without opening another physics client. */
async function waitForWeb() {
  const deadline = Date.now() + 300000;
  while (Date.now() < deadline) {
    try {
      const health = await readBridge('/health');
      if (health.runtimeConnected && health.backend === 'web') return;
    } catch (error) {
      console.log(`Waiting for web runtime: ${error.message}`);
    }
    await new Promise((resolve) => setTimeout(resolve, 5000));
  }
  throw new Error('No coordinated web runtime appeared within five minutes');
}

await waitForWeb();
const started = Date.now();
let previous;
while (Date.now() - started <= (durationSeconds + 5) * 1000) {
  const health = await readBridge('/health');
  const state = await readBridge('/state');
  const log = await readFile(logPath, 'utf8');
  assert.equal(health.backend, 'web');
  assert.equal(health.runtimeConnected, true);
  assert.ok(health.lastFrameAgeSec < 2, `Stale frames: ${health.lastFrameAgeSec}`);
  assert.equal(state.policy.loaded, false, 'Soak must run the default scripted controller');
  assert.ok(state.robot.base.position.every(Number.isFinite));
  assert.ok(!/Cannot enlarge memory|Simulation paused|RuntimeError|Aborted\(/.test(log), 'Browser log contains a runtime failure');
  if (previous) {
    assert.ok(state.simulationTime > previous.simulationTime, 'Simulation clock stopped');
    for (const kind of ['state', 'scan', 'pointcloud', 'camera']) {
      assert.ok(health.frames[kind] > previous.frames[kind], `${kind} stopped publishing`);
    }
  }
  const sample = { elapsed: (Date.now() - started) / 1000, simulationTime: state.simulationTime, frames: health.frames, lastFrameAgeSec: health.lastFrameAgeSec };
  samples.push(sample);
  console.log(JSON.stringify(sample));
  previous = sample;
  if (sample.elapsed >= durationSeconds) break;
  await new Promise((resolve) => setTimeout(resolve, 5000));
}
assert.ok(samples.at(-1).elapsed >= durationSeconds);
const cameraRate = (samples.at(-1).frames.camera - samples[0].frames.camera) / samples.at(-1).elapsed;
assert.ok(cameraRate >= 4, `Camera publishing averaged only ${cameraRate} Hz`);
await writeFile('/tmp/go2-web-soak.json', JSON.stringify({ durationSeconds, cameraRate, samples }, null, 2));
console.log(`PASS: ${durationSeconds}s actual web bridge soak; camera ${cameraRate.toFixed(2)} Hz`);
