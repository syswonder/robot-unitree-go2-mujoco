import { BaseController } from '../../../src/utils/controllers/BaseController.js';
import { PolicyController } from '../../../src/policy/PolicyController.js';

const TWO_PI = Math.PI * 2;
const LINK_LENGTH = 0.213;
const HOME_FOOT_Z = -2 * LINK_LENGTH * Math.cos(0.9);
const GAIT_FREQUENCY_HZ = 2.0;
const STANCE_FRACTION = 0.56;
const STRIDE_LENGTH = 0.13;
const LATERAL_STRIDE = 0.075;
const FOOT_CLEARANCE = 0.045;
const TWIST_LIMITS = Object.freeze([0.6, 0.35, 0.6]);
const POLICY_COMMAND_LIMITS = Object.freeze([0.6, 0.35, 1.0]);
const POLICY_FEEDFORWARD = Object.freeze([2.0, 2.0, 2.2]);
const POLICY_INTEGRAL_GAINS = Object.freeze([2.5, 2.5, 5.0]);
const WEB_COMMAND_WATCHDOG_S = 2.0;

export const GO2_LEGS = Object.freeze({
  FL: Object.freeze({ front: 1, side: 1, phaseOffset: 0 }),
  FR: Object.freeze({ front: 1, side: -1, phaseOffset: 0.5 }),
  RL: Object.freeze({ front: -1, side: 1, phaseOffset: 0.5 }),
  RR: Object.freeze({ front: -1, side: -1, phaseOffset: 0 })
});

export const GO2_ACTUATORS = Object.freeze(
  Object.keys(GO2_LEGS).flatMap((leg) => [`${leg}_hip`, `${leg}_thigh`, `${leg}_calf`])
);

const JOINT_GAINS = Object.freeze({
  hip: Object.freeze({ kp: 45, kd: 1.5, limit: 23.7 }),
  thigh: Object.freeze({ kp: 55, kd: 2.0, limit: 23.7 }),
  calf: Object.freeze({ kp: 60, kd: 2.0, limit: 45.43 })
});

function clamp(value, min, max) {
  return Math.max(min, Math.min(max, value));
}

function normalizeCycle(value) {
  return ((value % 1) + 1) % 1;
}

/** Decode MuJoCo's packed name table into stable joint and actuator addresses. */
function decodeNames(model, count, addressArray) {
  const decoder = new TextDecoder('utf-8');
  const names = new Map();
  for (let index = 0; index < count; index++) {
    const address = addressArray[index];
    let end = address;
    while (end < model.names.length && model.names[end] !== 0) end++;
    names.set(decoder.decode(model.names.subarray(address, end)), index);
  }
  return names;
}

/** Solve the existing Go2 leg geometry for bounded gait targets. */
function legInverseKinematics(x, y, z) {
  const sagittalZ = -Math.hypot(y, z);
  const distanceSquared = x * x + sagittalZ * sagittalZ;
  const cosineKnee = clamp(
    (distanceSquared - 2 * LINK_LENGTH * LINK_LENGTH) / (2 * LINK_LENGTH * LINK_LENGTH),
    -0.999,
    0.999
  );
  const knee = -Math.acos(cosineKnee);
  const legDirection = Math.atan2(-x, -sagittalZ);
  return {
    hip: clamp(Math.atan2(y, -z), -0.75, 0.75),
    thigh: clamp(legDirection - knee * 0.5, -0.45, 2.6),
    calf: clamp(knee, -2.65, -0.9)
  };
}

/** Translate optional development keys into normalized gait commands. */
export function computeGo2Command(keyStates) {
  if (keyStates.Space || keyStates.KeyX) return { forward: 0, turn: 0, lateral: 0 };
  return {
    forward: Number(Boolean(keyStates.KeyW)) - Number(Boolean(keyStates.KeyS)),
    turn: Number(Boolean(keyStates.KeyA)) - Number(Boolean(keyStates.KeyD)),
    lateral: Number(Boolean(keyStates.KeyQ)) - Number(Boolean(keyStates.KeyE))
  };
}

/** Generate one complete set of joint targets from a local velocity command. */
export function computeGo2Targets(command, phase) {
  const targets = {};
  const commandMagnitude = Math.min(1, Math.hypot(command.forward, command.turn, command.lateral));

  for (const [leg, config] of Object.entries(GO2_LEGS)) {
    if (commandMagnitude < 1e-6) {
      targets[`${leg}_hip`] = 0;
      targets[`${leg}_thigh`] = 0.9;
      targets[`${leg}_calf`] = -1.8;
      continue;
    }

    const cycle = normalizeCycle(phase / TWO_PI + config.phaseOffset);
    const inStance = cycle < STANCE_FRACTION;
    const progress = inStance
      ? cycle / STANCE_FRACTION
      : (cycle - STANCE_FRACTION) / (1 - STANCE_FRACTION);
    const travel = inStance ? 0.5 - progress : -0.5 + progress;
    const lift = inStance ? 0 : Math.sin(Math.PI * progress);
    const legForward = clamp(command.forward - config.side * command.turn * 0.65, -1, 1);
    const x = STRIDE_LENGTH * legForward * travel;
    const y = (LATERAL_STRIDE * command.lateral + 0.115 * config.front * command.turn) * travel;
    const z = HOME_FOOT_Z + FOOT_CLEARANCE * lift * commandMagnitude;
    const angles = legInverseKinematics(x, y, z);
    targets[`${leg}_hip`] = angles.hip;
    targets[`${leg}_thigh`] = angles.thigh;
    targets[`${leg}_calf`] = angles.calf;
  }
  return targets;
}

export class Go2Controller extends BaseController {
  /** Start with the scripted gait and no policy session or external velocity. */
  constructor(clock = () => performance.now() / 1000) {
    super();
    this.mujoco = null;
    this.joints = new Map();
    this.actuators = new Map();
    this.homeQpos = null;
    this.phase = 0;
    this.gaitBlend = 0;
    this.resetHeld = false;
    this.externalControlEnabled = false;
    this.twist = { linearX: 0, linearY: 0, angularZ: 0 };
    this.policy = new PolicyController();
    this.supportBuffer = null;
    this.clock = clock;
    this.lastTwistTime = -Infinity;
    this.velocityIntegral = new Float64Array(3);
    this.policyVelocityIntegral = new Float64Array(3);
  }

  /** Resolve the twelve driven joints and restore the scene's home keyframe. */
  async initialize(model, data, mujoco) {
    this.mujoco = mujoco;
    const jointNames = decodeNames(model, model.njnt, model.name_jntadr);
    const actuatorNames = decodeNames(model, model.nu, model.name_actuatoradr);
    const bodyNames = decodeNames(model, model.nbody, model.name_bodyadr);
    const geomNames = decodeNames(model, model.ngeom, model.name_geomadr);
    this.baseBody = bodyNames.get('base_link');
    this.feet = Object.keys(GO2_LEGS).map((leg) => ({ body: bodyNames.get(`${leg}_foot`), geom: geomNames.get(leg) }));
    this.supportBuffer = new mujoco.DoubleBuffer(model.nv);

    for (const actuator of GO2_ACTUATORS) {
      const jointName = `${actuator}_joint`;
      if (!jointNames.has(jointName) || !actuatorNames.has(actuator)) {
        throw new Error(`Go2 model is missing joint or actuator: ${actuator}`);
      }
      const jointIndex = jointNames.get(jointName);
      this.joints.set(actuator, {
        qpos: model.jnt_qposadr[jointIndex],
        qvel: model.jnt_dofadr[jointIndex]
      });
      this.actuators.set(actuator, actuatorNames.get(actuator));
    }

    if (model.nkey < 1 || model.key_qpos.length < model.nq) {
      throw new Error('Go2 model does not provide the required home keyframe');
    }
    this.homeQpos = Float64Array.from(model.key_qpos.slice(0, model.nq));
    this.reset(model, data);
    this.initialized = true;
  }

  /** Restore the configured spawn and discard velocity and observation history. */
  reset(_model, data) {
    if (!this.homeQpos) return;
    data.qpos.set(this.homeQpos);
    data.qvel.fill(0);
    data.ctrl.fill(0);
    if (data.qacc) data.qacc.fill(0);
    this.phase = 0;
    this.gaitBlend = 0;
    this.emergencyStop();
    this.policy.reset();
    if (this.mujoco) this.mujoco.mj_forward(_model, data);
  }

  /** Apply either explicit ONNX control or the default scripted gait at each physics step. */
  async step(keyStates, model, data, mujoco) {
    if (!this.initialized) await this.initialize(model, data, mujoco);

    if (keyStates.KeyX) {
      if (!this.resetHeld) this.reset(model, data);
      this.resetHeld = true;
    } else {
      this.resetHeld = false;
    }

    if (keyStates.Space || keyStates.KeyX) this.emergencyStop();
    const manual = ['KeyW', 'KeyS', 'KeyA', 'KeyD', 'KeyQ', 'KeyE', 'Space', 'KeyX'].some((key) => keyStates[key]);
    if (!manual && this.externalControlEnabled &&
        this.clock() - this.lastTwistTime > WEB_COMMAND_WATCHDOG_S) this.emergencyStop();
    const keys = computeGo2Command(keyStates);
    const velocity = !manual && this.externalControlEnabled ? this.twist : {
      linearX: keys.forward * 0.6, linearY: keys.lateral * 0.35, angularZ: keys.turn * 0.6
    };
    const policyVelocity = this.policy.status.loaded
      ? this.trackedPolicyCommand(velocity, model, data)
      : velocity;
    if (await this.policy.step(model, data, this.joints, this.actuators, policyVelocity)) return;
    if (!this.initialized) return;
    const rawCommand = keyStates.KeyX
      ? { forward: 0, turn: 0, lateral: 0 }
      : { forward: velocity.linearX / 0.6, lateral: velocity.linearY / 0.35, turn: velocity.angularZ / 0.6 };
    const active = Math.hypot(rawCommand.forward, rawCommand.turn, rawCommand.lateral) > 0;
    const timestep = model.opt.timestep;
    const blendRate = active ? 5.0 : 7.0;
    this.gaitBlend += (Number(active) - this.gaitBlend) * Math.min(1, blendRate * timestep);
    if (this.gaitBlend > 1e-3) this.phase += TWO_PI * GAIT_FREQUENCY_HZ * timestep;

    const command = this.scriptedCommand(velocity, model, data);
    const targets = computeGo2Targets(command, this.phase);
    const support = this.supportTorque(model, data);
    for (const actuator of GO2_ACTUATORS) {
      const joint = this.joints.get(actuator);
      const type = actuator.endsWith('_hip') ? 'hip' : actuator.endsWith('_thigh') ? 'thigh' : 'calf';
      const gains = JOINT_GAINS[type];
      const torque = gains.kp * (targets[actuator] - data.qpos[joint.qpos]) -
        gains.kd * data.qvel[joint.qvel] + support[joint.qvel];
      data.ctrl[this.actuators.get(actuator)] = clamp(torque, -gains.limit, gains.limit);
    }
  }

  /** Match native bounded velocity feedback using world linear velocity rotated into the base frame. */
  scriptedCommand(velocity, model, data) {
    const desired = [clamp(velocity.linearX, -0.4, 0.4), clamp(velocity.linearY, -0.15, 0.15), clamp(velocity.angularZ, -0.5, 0.5)];
    if (Math.hypot(...desired) < 1e-6) {
      this.velocityIntegral.fill(0);
      return { forward: 0, lateral: 0, turn: 0 };
    }
    const offset = this.baseBody * 9;
    const local = [0, 1].map((axis) => [0, 1, 2].reduce((sum, row) => sum + data.xmat[offset + row * 3 + axis] * data.qvel[row], 0));
    const measured = [...local, data.qvel[5]];
    const limits = [0.4, 0.15, 0.5];
    const gains = [4, 4, 2];
    const values = desired.map((value, i) => {
      const error = value - measured[i];
      const raw = value / limits[i] + this.velocityIntegral[i];
      if (Math.abs(raw) < 1 || raw * error < 0) this.velocityIntegral[i] += gains[i] * error * model.opt.timestep;
      return clamp(value / limits[i] + this.velocityIntegral[i], -1, 1) * this.gaitBlend;
    });
    return { forward: values[0], lateral: values[1], turn: values[2] };
  }

  /** Compensate policy response so low-speed Twist tracks measured body velocity. */
  trackedPolicyCommand(velocity, model, data) {
    const desired = [velocity.linearX, velocity.linearY, velocity.angularZ]
      .map((value, i) => clamp(value, -TWIST_LIMITS[i], TWIST_LIMITS[i]));
    const offset = this.baseBody * 9;
    const local = [0, 1].map((axis) => [0, 1, 2].reduce(
      (sum, row) => sum + data.xmat[offset + row * 3 + axis] * data.qvel[row], 0));
    const measured = [...local, data.qvel[5]];
    const result = desired.map((value, i) => {
      if (Math.abs(value) <= 1e-6) {
        this.policyVelocityIntegral[i] = 0;
        return 0;
      }
      const error = value - measured[i];
      const raw = POLICY_FEEDFORWARD[i] * value + this.policyVelocityIntegral[i];
      if (Math.abs(raw) < POLICY_COMMAND_LIMITS[i] || raw * error < 0) {
        this.policyVelocityIntegral[i] = clamp(
          this.policyVelocityIntegral[i] + POLICY_INTEGRAL_GAINS[i] * error * model.opt.timestep,
          -POLICY_COMMAND_LIMITS[i], POLICY_COMMAND_LIMITS[i]);
      }
      return clamp(POLICY_FEEDFORWARD[i] * value + this.policyVelocityIntegral[i],
        -POLICY_COMMAND_LIMITS[i], POLICY_COMMAND_LIMITS[i]);
    });
    return { linearX: result[0], linearY: result[1], angularZ: result[2] };
  }

  /** Support gravity through feet in floor contact using joint torques, matching the native controller. */
  supportTorque(model, data) {
    const contacts = new Set();
    // Embind copies the contact vector and each element; JS garbage collection does not release them.
    const contactVector = data.contact;
    try {
      for (let i = 0; i < contactVector.size(); i++) {
        const contact = contactVector.get(i);
        try {
          if (Math.abs(contact.frame[2]) > 0.5 && contact.dist < 0.003) {
            contacts.add(contact.geom[0]);
            contacts.add(contact.geom[1]);
          }
        } finally {
          contact.delete();
        }
      }
    } finally {
      contactVector.delete();
    }
    const support = this.feet.filter((foot) => contacts.has(foot.geom));
    const torque = this.supportBuffer.GetView();
    torque.fill(0);
    for (const foot of support) {
      const force = Array.from(model.opt.gravity, (value) => value * model.body_subtreemass[this.baseBody] / support.length);
      const point = Array.from(data.xpos.slice(foot.body * 3, foot.body * 3 + 3));
      this.mujoco.mj_applyFT(model, data, force, [0, 0, 0], point, foot.body, torque);
    }
    return torque;
  }

  async dispose() {
    this.initialized = false;
    await this.policy.unload();
    this.supportBuffer?.delete();
    this.supportBuffer = null;
  }

  setExternalControlEnabled(enabled) {
    this.externalControlEnabled = Boolean(enabled);
    if (!enabled) this.emergencyStop();
  }

  /** Accept finite continuous body-frame Twist components, preserving fractional commands. */
  setTwist(command) {
    const values = [command.linearX ?? 0, command.linearY ?? 0, command.angularZ ?? 0];
    if (!values.every((value) => typeof value === 'number' && Number.isFinite(value))) return false;
    this.twist = { linearX: clamp(values[0], -0.6, 0.6), linearY: clamp(values[1], -0.35, 0.35), angularZ: clamp(values[2], -0.6, 0.6) };
    this.lastTwistTime = this.clock();
    this.externalControlEnabled = true;
    return true;
  }

  emergencyStop() {
    this.twist = { linearX: 0, linearY: 0, angularZ: 0 };
    this.gaitBlend = 0;
    this.velocityIntegral.fill(0);
    this.policyVelocityIntegral.fill(0);
    return true;
  }

  /** Dispatch the shared explicit policy command; failed loads keep the scripted gait available. */
  async policyCommand(command, model) {
    if (command.id !== 'moe_rough' || !['load', 'unload'].includes(command.action)) {
      throw new Error('Expected policy moe_rough with action load or unload');
    }
    this.policyVelocityIntegral.fill(0);
    if (command.action === 'unload') { await this.policy.unload(); return true; }
    return this.policy.load(model);
  }

  /** Publish Go2 base and all twelve joint states using the shared bridge representation. */
  getRobotState(_model, data) {
    const addresses = GO2_ACTUATORS.map((name) => this.joints.get(name));
    return {
      base: {
        position: Array.from(data.qpos.slice(0, 3)), quaternion: Array.from(data.qpos.slice(3, 7)),
        linearVelocity: Array.from(data.qvel.slice(0, 3)), angularVelocity: Array.from(data.qvel.slice(3, 6))
      },
      joints: {
        names: GO2_ACTUATORS.map((name) => `${name}_joint`),
        positions: addresses.map((joint) => data.qpos[joint.qpos]),
        velocities: addresses.map((joint) => data.qvel[joint.qvel])
      },
      policy: { ...this.policy.status }
    };
  }

  getControlKeys() {
    return ['KeyW', 'KeyS', 'KeyA', 'KeyD', 'KeyQ', 'KeyE', 'Space', 'KeyX'];
  }

  /** Describe the optional keyboard controls in the developer panel. */
  getDescription() {
    return [
      'Move: W/S | Turn: A/D | Sidestep: Q/E',
      'Stand: Space | Reset pose: X'
    ].join('\n');
  }
}
