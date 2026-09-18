import test from 'node:test';
import assert from 'node:assert/strict';
import * as THREE from 'three';
import { createMujocoMeshGeometry, createMujocoColorTexture } from '../src/utils/MujocoGeometry.js';

/** Represent two triangles sharing positions but using separate corner normals and UVs, with nonzero offsets. */
function meshFixture() {
  return {
    mesh_facenum: [2], mesh_faceadr: [1], mesh_vertadr: [1], mesh_normaladr: [1],
    mesh_normalnum: [2], mesh_texcoordadr: [1], mesh_texcoordnum: [6],
    mesh_vert: new Float32Array([99, 99, 99, 1, 2, 3, 2, 2, 3, 2, 4, 3, 1, 4, 3]),
    mesh_normal: new Float32Array([99, 99, 99, 0, 0, 1, 0, 1, 0]),
    mesh_texcoord: new Float32Array([99, 99, 0, 0, 1, 0, 1, 1, 0.25, 0.5, 0.75, 0.5, 0, 1]),
    mesh_face: new Int32Array([99, 99, 99, 0, 1, 2, 0, 2, 3]),
    mesh_facenormal: new Int32Array([99, 99, 99, 0, 0, 0, 1, 1, 1]),
    mesh_facetexcoord: new Int32Array([99, 99, 99, 0, 1, 2, 3, 4, 5])
  };
}

/** Repeated rendering must retain physical geometry and distinct UV/normal values at shared vertex seams. */
test('triangle expansion preserves MuJoCo arrays and independent corner indices', () => {
  const model = meshFixture();
  const original = structuredClone(model);
  const geometry = createMujocoMeshGeometry(model, 0);
  assert.equal(geometry.index, null);
  assert.deepEqual(Array.from(geometry.attributes.position.array), [1, 3, -2, 2, 3, -2, 2, 3, -4, 1, 3, -2, 2, 3, -4, 1, 3, -4]);
  assert.deepEqual(Array.from(geometry.attributes.uv.array), [0, 0, 1, 0, 1, 1, 0.25, 0.5, 0.75, 0.5, 0, 1]);
  assert.deepEqual(Array.from(geometry.attributes.normal.array), [0, 1, -0, 0, 1, -0, 0, 1, -0, 0, 0, -1, 0, 0, -1, 0, 0, -1]);
  assert.deepEqual(model, original);
  geometry.attributes.position.array[0] = 500;
  const rebuilt = createMujocoMeshGeometry(model, 0);
  assert.equal(rebuilt.attributes.position.array[0], 1);
  assert.deepEqual(model, original);
  geometry.dispose();
  rebuilt.dispose();
});

/** Meshes without texture coordinates or imported normals must still produce finite render geometry. */
test('untextured meshes generate normals without reading negative attribute offsets', () => {
  const model = meshFixture();
  model.mesh_texcoordnum[0] = 0;
  model.mesh_texcoordadr[0] = -1;
  model.mesh_normalnum[0] = 0;
  model.mesh_normaladr[0] = -1;
  model.mesh_facenormal.fill(-1);
  const geometry = createMujocoMeshGeometry(model, 0);
  assert.equal(geometry.attributes.uv, undefined);
  assert.ok(geometry.attributes.normal.array.every(Number.isFinite));
  assert.ok(geometry.attributes.normal.array.some((value) => value !== 0));
  geometry.dispose();
});

/** Texture ID two must obey its material and retain filtered sRGB sampling, not a special repetition rule. */
test('color textures use material repeats, sRGB, and linear mipmaps', () => {
  const model = {
    tex_width: [0, 0, 2], tex_height: [0, 0, 1], tex_adr: [0, 0, 2], tex_nchannel: [0, 0, 3],
    tex_data: new Uint8Array([99, 99, 10, 20, 30, 40, 50, 60]), mat_texrepeat: [1.5, 2]
  };
  const texture = createMujocoColorTexture(model, 2, 0);
  assert.deepEqual(texture.repeat.toArray(), [1.5, 2]);
  assert.deepEqual(Array.from(texture.image.data), [10, 20, 30, 255, 40, 50, 60, 255]);
  assert.equal(texture.colorSpace, THREE.SRGBColorSpace);
  assert.equal(texture.magFilter, THREE.LinearFilter);
  assert.equal(texture.minFilter, THREE.LinearMipmapLinearFilter);
  assert.equal(texture.generateMipmaps, true);
  texture.dispose();
});
