import {
  MOE_ROUGH_DEFAULT_JOINT_POS, MOE_ROUGH_JOINT_NAMES,
  MoeRoughHistory, buildMoeRoughObservation
} from '../../assets/robots/go2/policy/moe_rough/contract.js';

/** Run the upstream MoE Rough observation and PD contract without changing the pose on load. */
export class PolicyController {
  /** Start unloaded; runtimeFactory permits testing session failures without downloading WASM. */
  constructor(runtimeFactory = () => import('../../node_modules/onnxruntime-web/dist/ort.wasm.min.mjs')) {
    this.runtimeFactory = runtimeFactory;
    this.status = { id: 'moe_rough', loaded: false, error: null };
    this.generation = 0;
    this.session = null;
    this.pending = null;
    this.history = new MoeRoughHistory();
    this.lastAction = new Float32Array(12);
    this.targets = null;
    this.nextTime = 0;
  }

  /** Explicitly fetch and validate the released config and model; cancelled loads release their session. */
  async load(model) {
    const release = this.unload();
    const generation = this.generation;
    await release;
    let session;
    try {
      const url = new URL('../../assets/robots/go2/policy/moe_rough/policy.json', import.meta.url);
      const response = await fetch(url);
      if (!response.ok) throw new Error(`Policy config HTTP ${response.status}`);
      const config = await response.json();
      if (config.control_dt !== 0.02 || config.stiffness !== 20 || config.damping !== 0.5 ||
          config.action_scale !== 0.25 || config.control_type !== 'joint_position' ||
          config.policy_joint_names.join(',') !== MOE_ROUGH_JOINT_NAMES.join(',') ||
          config.default_joint_pos.length !== 12 ||
          config.default_joint_pos.some((value, i) => !Number.isFinite(value) || Math.abs(value - MOE_ROUGH_DEFAULT_JOINT_POS[i]) > 1e-6)) {
        throw new Error('MoE Rough config does not match the released 50 Hz contract');
      }
      const decimation = 0.02 / model.opt.timestep;
      if (decimation < 1 || Math.abs(decimation - Math.round(decimation)) > 1e-6) {
        throw new Error('MoE Rough needs a physics timestep that divides 0.02 seconds');
      }
      const ort = await this.runtimeFactory();
      ort.env.wasm.numThreads = 1;
      const weights = await fetch(new URL(config.onnx.path, url));
      if (!weights.ok) throw new Error(`Policy model HTTP ${weights.status}`);
      session = await ort.InferenceSession.create(await weights.arrayBuffer(), {
        executionProviders: ['wasm'], graphOptimizationLevel: 'all'
      });
      if (session.inputNames.join(',') !== 'history' || session.outputNames.join(',') !== 'actions,weights,latent') {
        throw new Error('MoE Rough ONNX input/output names do not match the released contract');
      }
      if (generation !== this.generation) { await session.release(); return false; }
      this.ort = ort;
      this.session = session;
      this.reset();
      this.status = { id: 'moe_rough', loaded: true, error: null };
      return true;
    } catch (error) {
      if (session) await session.release();
      if (generation === this.generation) this.status = { id: 'moe_rough', loaded: false, error: String(error.message ?? error) };
      throw error;
    }
  }

  /** Stop applying policy immediately, then release WASM after any in-flight inference finishes. */
  async unload() {
    this.generation++;
    const session = this.session;
    this.session = null;
    this.status = { id: 'moe_rough', loaded: false, error: null };
    this.reset();
    await this.pending?.catch(() => {});
    if (session) await session.release();
  }

  /** Invalidate pending actions, clear observations, and force fresh inference after a pose reset. */
  reset() {
    this.generation++;
    this.history.reset();
    this.lastAction.fill(0);
    this.targets = null;
    this.nextTime = 0;
  }

  /** Infer at 50 Hz of simulation time, await completion, and apply PD at every physics substep. */
  async step(model, data, joints, actuators, command) {
    if (!this.status.loaded) return false;
    const generation = this.generation;
    try {
      if (data.time + 1e-9 >= this.nextTime) {
        const addresses = MOE_ROUGH_JOINT_NAMES.map((name) => joints.get(name.replace('_joint', '')));
        this.history.append(buildMoeRoughObservation({
          baseAngularVelocity: data.qvel.slice(3, 6), quaternionWxyz: data.qpos.slice(3, 7),
          command: [command.linearX, command.linearY, command.angularZ],
          jointPosition: addresses.map((joint) => data.qpos[joint.qpos]),
          jointVelocity: addresses.map((joint) => data.qvel[joint.qvel]), lastAction: this.lastAction
        }));
        this.pending = this.session.run({ history: new this.ort.Tensor('float32', this.history.toArray(), [1, 225]) });
        const outputs = await this.pending;
        if (generation !== this.generation) return false;
        if (outputs.actions?.data.length !== 12 || !outputs.actions.data.every(Number.isFinite)) {
          throw new Error('MoE Rough ONNX output actions is invalid');
        }
        this.lastAction.set(outputs.actions.data);
        this.targets = MOE_ROUGH_DEFAULT_JOINT_POS.map((position, i) => position + 0.25 * this.lastAction[i]);
        this.nextTime = Number(data.time) + 0.02;
      }
      MOE_ROUGH_JOINT_NAMES.forEach((name, i) => {
        const actuator = name.replace('_joint', '');
        const joint = joints.get(actuator);
        const index = actuators.get(actuator);
        const torque = 20 * (this.targets[i] - data.qpos[joint.qpos]) - 0.5 * data.qvel[joint.qvel];
        data.ctrl[index] = Math.max(model.actuator_ctrlrange[index * 2], Math.min(model.actuator_ctrlrange[index * 2 + 1], torque));
      });
      return true;
    } catch (error) {
      if (generation === this.generation) {
        await this.unload();
        this.status.error = `Policy inference failed: ${error.message ?? error}`;
      }
      return false;
    } finally {
      this.pending = null;
    }
  }
}
