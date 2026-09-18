import {
  MOE_ROUGH_AUTO_MOTION,
  MOE_ROUGH_DEFAULT_JOINT_POS,
  MOE_ROUGH_HISTORY_DIM,
  MOE_ROUGH_JOINT_NAMES,
  MoeRoughHistory,
  buildMoeRoughObservation,
  computeMoeRoughCommand,
  wrapAngle,
  yawFromQuaternion
} from './contract.js';

const PHYSICS_TIMESTEP = 0.002;
const MANUAL_KEYS = Object.freeze(['KeyW', 'KeyS', 'KeyA', 'KeyD', 'KeyQ', 'KeyE', 'Space']);

function clamp(value, minimum, maximum) {
  return Math.max(minimum, Math.min(maximum, value));
}

/** Browser runner for the blind, omnidirectional Go2 MoE rough-terrain policy. */
export class Go2MoeRoughPolicyRunner {
  constructor(config, options) {
    this.config = config;
    this.model = options.model;
    this.data = options.data;
    this.mujoco = options.mujoco;
    this.configUrl = options.configUrl;
    this.getKeyStates = options.getKeyStates ?? (() => ({}));
    this.jointQposAddresses = options.qposAddresses;
    this.jointQvelAddresses = options.qvelAddresses;
    this.lastAction = new Float32Array(MOE_ROUGH_JOINT_NAMES.length);
    this.command = new Float32Array(3);
    this.history = new MoeRoughHistory();
    this.controlStep = 0;
    this.autoRoute = null;
  }

  async init() {
    if (Math.abs(this.model.opt.timestep - PHYSICS_TIMESTEP) > 1e-9) {
      throw new Error('Go2 MoE Rough requires a 0.002 s physics timestep; select the Go2 MoE Stairs environment');
    }
    const ort = globalThis.ort;
    if (!ort?.InferenceSession || !ort?.Tensor) throw new Error('ONNX Runtime Web is unavailable');
    const modelUrl = new URL(this.config.onnx.path, this.configUrl).href;
    const response = await fetch(modelUrl);
    if (!response.ok) throw new Error(`Failed to load MoE Rough ONNX model: ${response.status}`);
    this.ort = ort;
    this.session = await ort.InferenceSession.create(await response.arrayBuffer(), {
      executionProviders: ['wasm'],
      graphOptimizationLevel: 'all'
    });
    if (this.session.inputNames.join(',') !== 'history' ||
        this.session.outputNames.join(',') !== 'actions,weights,latent') {
      throw new Error('MoE Rough ONNX input/output names do not match the released contract');
    }
    this.resetSimulation();
    this.reset(this._readState());
  }

  _readState() {
    return {
      rootPos: Float32Array.from(this.data.qpos.subarray(0, 3)),
      rootQuat: Float32Array.from(this.data.qpos.subarray(3, 7)),
      rootAngVel: Float32Array.from(this.data.qvel.subarray(3, 6))
    };
  }

  resetSimulation() {
    const rootX = this.data.qpos[0];
    const rootY = this.data.qpos[1];
    this.data.qpos.fill(0);
    this.data.qpos.set([rootX, rootY, 0.42, 1, 0, 0, 0], 0);
    for (let index = 0; index < MOE_ROUGH_JOINT_NAMES.length; index++) {
      this.data.qpos[this.jointQposAddresses[index]] = MOE_ROUGH_DEFAULT_JOINT_POS[index];
    }
    this.data.qvel.fill(0);
    this.data.ctrl.fill(0);
    this.data.qacc?.fill(0);
    this.mujoco.mj_forward(this.model, this.data);
  }

  reset() {
    this.lastAction.fill(0);
    this.command.fill(0);
    this.history.reset();
    this.controlStep = 0;
    this.autoRoute = null;
  }

  _automaticCommand(state) {
    const route = this.autoRoute;
    const routeConfig = this.config.moe_rough.automatic_route;
    const currentYaw = yawFromQuaternion(state.rootQuat);
    const lateralError = state.rootPos[1] - route.laneY;

    if (route.phase === 'outbound' && state.rootPos[0] >= routeConfig.far_x) {
      route.phase = 'turnaround';
    }
    if (route.phase === 'turnaround') {
      const targetYaw = wrapAngle(route.outboundYaw + Math.PI);
      const yawError = wrapAngle(targetYaw - currentYaw);
      if (Math.abs(yawError) <= routeConfig.heading_tolerance) route.phase = 'inbound';
      else return new Float32Array([0, 0, clamp(
        routeConfig.turn_gain * yawError,
        -routeConfig.turn_rate,
        routeConfig.turn_rate
      )]);
    }
    if (route.phase === 'inbound' && state.rootPos[0] <= routeConfig.near_x) {
      route.phase = 'complete';
    }
    if (route.phase === 'complete') return new Float32Array(3);

    const targetYaw = route.phase === 'outbound'
      ? route.outboundYaw
      : wrapAngle(route.outboundYaw + Math.PI);
    const yawError = wrapAngle(targetYaw - currentYaw);
    const lateralCorrection = route.phase === 'outbound'
      ? -routeConfig.lateral_gain * lateralError
      : routeConfig.lateral_gain * lateralError;
    return new Float32Array([
      routeConfig.forward_speed,
      0,
      clamp(
        routeConfig.heading_gain * yawError + lateralCorrection,
        -routeConfig.maximum_yaw_rate,
        routeConfig.maximum_yaw_rate
      )
    ]);
  }

  async step(state) {
    const keyStates = this.getKeyStates();
    if (MANUAL_KEYS.some((code) => Boolean(keyStates[code]))) this.autoRoute = null;
    this.command = this.autoRoute
      ? this._automaticCommand(state)
      : computeMoeRoughCommand(keyStates, this.config.moe_rough.command);

    const jointPosition = new Float32Array(MOE_ROUGH_JOINT_NAMES.length);
    const jointVelocity = new Float32Array(MOE_ROUGH_JOINT_NAMES.length);
    for (let index = 0; index < MOE_ROUGH_JOINT_NAMES.length; index++) {
      jointPosition[index] = this.data.qpos[this.jointQposAddresses[index]];
      jointVelocity[index] = this.data.qvel[this.jointQvelAddresses[index]];
    }
    const observation = buildMoeRoughObservation({
      baseAngularVelocity: state.rootAngVel,
      quaternionWxyz: state.rootQuat,
      command: this.command,
      jointPosition,
      jointVelocity,
      lastAction: this.lastAction
    });
    this.history.append(observation);
    const outputs = await this.session.run({
      history: new this.ort.Tensor('float32', this.history.toArray(), [1, MOE_ROUGH_HISTORY_DIM])
    });
    if (!outputs.actions || outputs.actions.data.length !== MOE_ROUGH_JOINT_NAMES.length ||
        !outputs.actions.data.every(Number.isFinite)) {
      throw new Error('MoE Rough ONNX output actions is invalid');
    }
    this.lastAction.set(outputs.actions.data);
    this.controlStep++;
    const targets = new Float32Array(MOE_ROUGH_JOINT_NAMES.length);
    for (let index = 0; index < targets.length; index++) {
      targets[index] = MOE_ROUGH_DEFAULT_JOINT_POS[index] + this.config.action_scale * this.lastAction[index];
    }
    return targets;
  }

  getAvailableMotions() {
    return [MOE_ROUGH_AUTO_MOTION];
  }

  requestMotion(name, state) {
    if (name !== MOE_ROUGH_AUTO_MOTION) return false;
    this.autoRoute = {
      phase: 'outbound',
      laneY: state.rootPos[1],
      outboundYaw: yawFromQuaternion(state.rootQuat)
    };
    return true;
  }

  getPlaybackState() {
    return {
      command: Array.from(this.command),
      controlStep: this.controlStep,
      automaticPhase: this.autoRoute?.phase ?? null
    };
  }
}
