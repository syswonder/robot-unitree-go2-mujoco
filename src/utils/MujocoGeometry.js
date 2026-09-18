import * as THREE from 'three';

/** Expand independently indexed MuJoCo triangle corners without modifying WASM-owned source arrays. */
export function createMujocoMeshGeometry(model, meshId) {
  const cornerCount = model.mesh_facenum[meshId] * 3;
  const faceOffset = model.mesh_faceadr[meshId] * 3;
  const vertexOffset = model.mesh_vertadr[meshId] * 3;
  const normalOffset = model.mesh_normaladr[meshId] * 3;
  const uvOffset = model.mesh_texcoordadr[meshId] * 2;
  const positions = new Float32Array(cornerCount * 3);
  const normals = new Float32Array(cornerCount * 3);
  const uvs = model.mesh_texcoordnum[meshId] > 0 ? new Float32Array(cornerCount * 2) : null;
  let completeNormals = model.mesh_normalnum[meshId] > 0;
  for (let corner = 0; corner < cornerCount; corner++) {
    const faceIndex = faceOffset + corner;
    const vertex = vertexOffset + model.mesh_face[faceIndex] * 3;
    positions[corner * 3] = model.mesh_vert[vertex];
    positions[corner * 3 + 1] = model.mesh_vert[vertex + 2];
    positions[corner * 3 + 2] = -model.mesh_vert[vertex + 1];
    const normalIndex = model.mesh_facenormal[faceIndex];
    if (normalIndex >= 0 && model.mesh_normalnum[meshId] > 0) {
      const normal = normalOffset + normalIndex * 3;
      normals[corner * 3] = model.mesh_normal[normal];
      normals[corner * 3 + 1] = model.mesh_normal[normal + 2];
      normals[corner * 3 + 2] = -model.mesh_normal[normal + 1];
    } else {
      completeNormals = false;
    }
    const uvIndex = model.mesh_facetexcoord[faceIndex];
    if (uvs && uvIndex >= 0) {
      uvs[corner * 2] = model.mesh_texcoord[uvOffset + uvIndex * 2];
      uvs[corner * 2 + 1] = model.mesh_texcoord[uvOffset + uvIndex * 2 + 1];
    }
  }
  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
  if (completeNormals) geometry.setAttribute('normal', new THREE.BufferAttribute(normals, 3));
  else geometry.computeVertexNormals();
  if (uvs) geometry.setAttribute('uv', new THREE.BufferAttribute(uvs, 2));
  return geometry;
}

/** Decode a color texture using material repeat values and filtered sRGB sampling at every distance. */
export function createMujocoColorTexture(model, textureId, materialId) {
  const width = model.tex_width[textureId];
  const height = model.tex_height[textureId];
  const offset = model.tex_adr[textureId];
  const channels = model.tex_nchannel[textureId];
  const pixels = new Uint8Array(width * height * 4);
  for (let pixel = 0; pixel < width * height; pixel++) {
    const source = offset + pixel * channels;
    pixels[pixel * 4] = model.tex_data[source];
    pixels[pixel * 4 + 1] = channels > 1 ? model.tex_data[source + 1] : pixels[pixel * 4];
    pixels[pixel * 4 + 2] = channels > 2 ? model.tex_data[source + 2] : pixels[pixel * 4];
    pixels[pixel * 4 + 3] = channels > 3 ? model.tex_data[source + 3] : 255;
  }
  const texture = new THREE.DataTexture(pixels, width, height, THREE.RGBAFormat, THREE.UnsignedByteType);
  texture.repeat.set(model.mat_texrepeat[materialId * 2], model.mat_texrepeat[materialId * 2 + 1]);
  texture.wrapS = THREE.RepeatWrapping;
  texture.wrapT = THREE.RepeatWrapping;
  texture.colorSpace = THREE.SRGBColorSpace;
  texture.magFilter = THREE.LinearFilter;
  texture.minFilter = THREE.LinearMipmapLinearFilter;
  texture.generateMipmaps = true;
  texture.needsUpdate = true;
  return texture;
}
