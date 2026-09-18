export const PIE_JOINT_NAMES = Object.freeze([
  'FL_hip_joint', 'FL_thigh_joint', 'FL_calf_joint',
  'FR_hip_joint', 'FR_thigh_joint', 'FR_calf_joint',
  'RL_hip_joint', 'RL_thigh_joint', 'RL_calf_joint',
  'RR_hip_joint', 'RR_thigh_joint', 'RR_calf_joint'
]);

export const PIE_DEFAULT_JOINT_POS = new Float32Array([
  -0.1, 0.9, -1.8,
  0.1, 0.9, -1.8,
  -0.1, 0.9, -1.8,
  0.1, 0.9, -1.8
]);

export const PIE_OBSERVATION_TERM_DIMS = Object.freeze([3, 3, 3, 12, 12, 12]);
export const PIE_OBSERVATION_DIM = 45;
export const PIE_PROPRIO_HISTORY_LENGTH = 10;
export const PIE_DEPTH_RAW_WIDTH = 106;
export const PIE_DEPTH_HEIGHT = 60;
export const PIE_DEPTH_CROP = 10;
export const PIE_DEPTH_WIDTH = 86;
export const PIE_DEPTH_HISTORY_LENGTH = 2;
export const PIE_DEPTH_MIN_METERS = 0.05;
export const PIE_DEPTH_MAX_METERS = 3.0;

const GAUSSIAN_KERNEL = new Float32Array([
  0.07511361, 0.12384140, 0.07511361,
  0.12384140, 0.20417996, 0.12384140,
  0.07511361, 0.12384140, 0.07511361
]);

function normalizedQuaternionWxyz(quaternion) {
  const norm = Math.hypot(quaternion[0], quaternion[1], quaternion[2], quaternion[3]);
  if (!Number.isFinite(norm) || norm <= 1e-12) throw new Error('PIE base quaternion is invalid');
  return quaternion.map((value) => value / norm);
}

/** Project the world down vector into the robot base frame. */
export function projectedGravity(quaternionWxyz) {
  const [w, x, y, z] = normalizedQuaternionWxyz(Array.from(quaternionWxyz));
  return new Float32Array([
    2 * (y * w - x * z),
    -2 * (y * z + x * w),
    2 * (x * x + y * y) - 1
  ]);
}

/** Build the exact clean 45-D observation used by the released PIE actor. */
export function buildPieProprioception({
  baseAngularVelocity,
  quaternionWxyz,
  command,
  jointPosition,
  jointVelocity,
  lastAction
}) {
  const gravity = projectedGravity(quaternionWxyz);
  const observation = new Float32Array(PIE_OBSERVATION_DIM);
  observation.set(baseAngularVelocity, 0);
  observation.set(gravity, 3);
  observation.set(command, 6);
  for (let index = 0; index < 12; index++) {
    observation[9 + index] = jointPosition[index] - PIE_DEFAULT_JOINT_POS[index];
    observation[21 + index] = jointVelocity[index];
    observation[33 + index] = lastAction[index];
  }
  if (!observation.every(Number.isFinite)) throw new Error('PIE proprioception contains non-finite values');
  return observation;
}

export class PieProprioHistory {
  constructor(length = PIE_PROPRIO_HISTORY_LENGTH) {
    this.length = length;
    this.frames = [];
  }

  reset() {
    this.frames = [];
  }

  append(observation) {
    const frame = Float32Array.from(observation);
    if (frame.length !== PIE_OBSERVATION_DIM || !frame.every(Number.isFinite)) {
      throw new Error('Invalid PIE proprioception history frame');
    }
    if (this.frames.length === 0) {
      this.frames = Array.from({ length: this.length }, () => frame.slice());
    } else {
      this.frames.push(frame);
      if (this.frames.length > this.length) this.frames.shift();
    }
  }

  toArray() {
    if (this.frames.length !== this.length) throw new Error('PIE proprioception history is empty');
    const result = new Float32Array(PIE_OBSERVATION_DIM * this.length);
    let outputOffset = 0;
    let termOffset = 0;
    for (const termDim of PIE_OBSERVATION_TERM_DIMS) {
      for (const frame of this.frames) {
        result.set(frame.subarray(termOffset, termOffset + termDim), outputOffset);
        outputOffset += termDim;
      }
      termOffset += termDim;
    }
    return result;
  }
}

function reflectedIndex(index, length) {
  if (index < 0) return 1;
  if (index >= length) return length - 2;
  return index;
}

/** Crop, blur, clamp, and normalize one Euclidean ray-depth frame. */
export function preprocessPieRayDepth(rawDepth) {
  if (rawDepth.length !== PIE_DEPTH_RAW_WIDTH * PIE_DEPTH_HEIGHT) {
    throw new Error(`PIE depth frame must be ${PIE_DEPTH_RAW_WIDTH}x${PIE_DEPTH_HEIGHT}`);
  }
  const cropped = new Float32Array(PIE_DEPTH_WIDTH * PIE_DEPTH_HEIGHT);
  for (let row = 0; row < PIE_DEPTH_HEIGHT; row++) {
    for (let column = 0; column < PIE_DEPTH_WIDTH; column++) {
      const value = rawDepth[row * PIE_DEPTH_RAW_WIDTH + column + PIE_DEPTH_CROP];
      cropped[row * PIE_DEPTH_WIDTH + column] = Number.isFinite(value) && value > 0
        ? value
        : PIE_DEPTH_MAX_METERS;
    }
  }

  const normalized = new Float32Array(cropped.length);
  for (let row = 0; row < PIE_DEPTH_HEIGHT; row++) {
    for (let column = 0; column < PIE_DEPTH_WIDTH; column++) {
      let blurred = 0;
      for (let kernelRow = -1; kernelRow <= 1; kernelRow++) {
        const sourceRow = reflectedIndex(row + kernelRow, PIE_DEPTH_HEIGHT);
        for (let kernelColumn = -1; kernelColumn <= 1; kernelColumn++) {
          const sourceColumn = reflectedIndex(column + kernelColumn, PIE_DEPTH_WIDTH);
          const kernelIndex = (kernelRow + 1) * 3 + kernelColumn + 1;
          blurred += cropped[sourceRow * PIE_DEPTH_WIDTH + sourceColumn] * GAUSSIAN_KERNEL[kernelIndex];
        }
      }
      normalized[row * PIE_DEPTH_WIDTH + column] = Math.min(
        PIE_DEPTH_MAX_METERS,
        Math.max(PIE_DEPTH_MIN_METERS, blurred)
      ) / PIE_DEPTH_MAX_METERS;
    }
  }
  return normalized;
}

export class PieDepthHistory {
  constructor(length = PIE_DEPTH_HISTORY_LENGTH) {
    this.length = length;
    this.frames = [];
  }

  reset() {
    this.frames = [];
  }

  append(frame) {
    const value = Float32Array.from(frame);
    if (value.length !== PIE_DEPTH_HEIGHT * PIE_DEPTH_WIDTH || !value.every(Number.isFinite)) {
      throw new Error('Invalid PIE depth history frame');
    }
    if (this.frames.length === 0) {
      this.frames = Array.from({ length: this.length }, () => value.slice());
    } else {
      this.frames.push(value);
      if (this.frames.length > this.length) this.frames.shift();
    }
  }

  toArray() {
    if (this.frames.length !== this.length) throw new Error('PIE depth history is empty');
    const result = new Float32Array(this.length * PIE_DEPTH_HEIGHT * PIE_DEPTH_WIDTH);
    this.frames.forEach((frame, index) => result.set(frame, index * frame.length));
    return result;
  }
}

/** Convert browser key state into the command range used during PIE training. */
export function computePieCommand(keyStates, forwardSpeed = 0.6, yawRate = 0.6) {
  if (keyStates.Space || keyStates.KeyS) return new Float32Array([0, 0, 0]);
  return new Float32Array([
    keyStates.KeyW ? forwardSpeed : 0,
    0,
    (Number(Boolean(keyStates.KeyA)) - Number(Boolean(keyStates.KeyD))) * yawRate
  ]);
}
