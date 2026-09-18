import { projectedGravity } from '../contract.js';

export const MOE_ROUGH_JOINT_NAMES = Object.freeze([
  'FL_hip_joint', 'FL_thigh_joint', 'FL_calf_joint',
  'FR_hip_joint', 'FR_thigh_joint', 'FR_calf_joint',
  'RL_hip_joint', 'RL_thigh_joint', 'RL_calf_joint',
  'RR_hip_joint', 'RR_thigh_joint', 'RR_calf_joint'
]);

export const MOE_ROUGH_DEFAULT_JOINT_POS = new Float32Array([
  0.1, 0.8, -1.5,
  -0.1, 0.8, -1.5,
  0.1, 1.0, -1.5,
  -0.1, 1.0, -1.5
]);

export const MOE_ROUGH_OBSERVATION_DIM = 45;
export const MOE_ROUGH_HISTORY_LENGTH = 5;
export const MOE_ROUGH_HISTORY_DIM = MOE_ROUGH_OBSERVATION_DIM * MOE_ROUGH_HISTORY_LENGTH;
export const MOE_ROUGH_AUTO_MOTION = 'Automatic Stair Round Trip';

/** Build one 45-D proprioceptive frame using the released MuJoCo deployment scales. */
export function buildMoeRoughObservation({
  baseAngularVelocity,
  quaternionWxyz,
  command,
  jointPosition,
  jointVelocity,
  lastAction
}) {
  const observation = new Float32Array(MOE_ROUGH_OBSERVATION_DIM);
  for (let index = 0; index < 3; index++) {
    observation[index] = baseAngularVelocity[index] * 0.25;
  }
  observation.set(projectedGravity(quaternionWxyz), 3);
  observation.set([command[0] * 2, command[1] * 2, command[2] * 0.25], 6);
  for (let index = 0; index < MOE_ROUGH_JOINT_NAMES.length; index++) {
    observation[9 + index] = jointPosition[index] - MOE_ROUGH_DEFAULT_JOINT_POS[index];
    observation[21 + index] = jointVelocity[index] * 0.05;
    observation[33 + index] = lastAction[index];
  }
  if (!observation.every(Number.isFinite)) throw new Error('MoE Rough observation contains non-finite values');
  return observation;
}

/** Maintain the same zero-filled five-frame history used by the released TorchScript runner. */
export class MoeRoughHistory {
  constructor() {
    this.values = new Float32Array(MOE_ROUGH_HISTORY_DIM);
  }

  reset() {
    this.values.fill(0);
  }

  append(observation) {
    if (observation.length !== MOE_ROUGH_OBSERVATION_DIM || !observation.every(Number.isFinite)) {
      throw new Error('Invalid MoE Rough observation frame');
    }
    this.values.copyWithin(0, MOE_ROUGH_OBSERVATION_DIM);
    this.values.set(observation, MOE_ROUGH_HISTORY_DIM - MOE_ROUGH_OBSERVATION_DIM);
  }

  toArray() {
    return this.values.slice();
  }
}

/** Map the standard Go2 keys to the policy's trained omnidirectional velocity commands. */
export function computeMoeRoughCommand(keyStates, config) {
  if (keyStates.Space) return new Float32Array(3);
  const forward = Number(Boolean(keyStates.KeyW)) - Number(Boolean(keyStates.KeyS));
  const lateral = Number(Boolean(keyStates.KeyQ)) - Number(Boolean(keyStates.KeyE));
  const yaw = Number(Boolean(keyStates.KeyA)) - Number(Boolean(keyStates.KeyD));
  return new Float32Array([
    forward * config.forward_speed,
    lateral * config.lateral_speed,
    yaw * config.yaw_rate
  ]);
}

export function yawFromQuaternion([w, x, y, z]) {
  return Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
}

export function wrapAngle(angle) {
  return Math.atan2(Math.sin(angle), Math.cos(angle));
}
