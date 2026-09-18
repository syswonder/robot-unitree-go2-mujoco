import * as THREE from 'three';

function finiteVector(value, name) {
  if (!Array.isArray(value) || value.length !== 3 || value.some((item) => !Number.isFinite(item))) {
    throw new Error(`${name} must be an array of three finite numbers`);
  }
  return value;
}

function finiteQuaternion(value, name) {
  if (!Array.isArray(value) || value.length !== 4 || value.some((item) => !Number.isFinite(item))) {
    throw new Error(`${name} must be an array of four finite numbers`);
  }
  if (Math.abs(Math.hypot(...value) - 1) > 1e-5) throw new Error(`${name} must be normalized`);
  return value;
}

/** Validate generated metadata before it can affect the rendered scene. */
export function validateSplatTransform(metadata) {
  if (!metadata || metadata.schemaVersion !== 1 || !metadata.transform) {
    throw new Error('Unsupported or missing scene transform metadata');
  }
  const { rotationDegrees, rotationQuaternion, scale, position } = metadata.transform;
  if (rotationQuaternion === undefined) finiteVector(rotationDegrees, 'rotationDegrees');
  else finiteQuaternion(rotationQuaternion, 'rotationQuaternion');
  finiteVector(position, 'position');
  if (!Number.isFinite(scale) || scale <= 0) {
    throw new Error('scale must be a finite number greater than zero');
  }
  return metadata.transform;
}

/** Apply the offline transform in Three.js object transform order. */
export function applySplatTransform(splatMesh, metadata) {
  const transform = validateSplatTransform(metadata);
  if (transform.rotationQuaternion) {
    splatMesh.quaternion.fromArray(transform.rotationQuaternion).normalize();
  } else {
    const [rotationX, rotationY, rotationZ] = transform.rotationDegrees;
    splatMesh.rotation.set(
      THREE.MathUtils.degToRad(rotationX),
      THREE.MathUtils.degToRad(rotationY),
      THREE.MathUtils.degToRad(rotationZ)
    );
  }
  splatMesh.scale.setScalar(transform.scale);
  splatMesh.position.fromArray(transform.position);
  splatMesh.updateMatrixWorld(true);
  return transform;
}
