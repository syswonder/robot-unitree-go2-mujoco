# SceneSmith scene_185

This runtime package is derived from `House/scene_185.tar` in
`nepfaff/scenesmith-example-scenes`.

- Source: https://huggingface.co/datasets/nepfaff/scenesmith-example-scenes
- SceneSmith project: https://github.com/nepfaff/scenesmith
- License: Apache-2.0 (as declared by the source dataset)

`scripts/prepare-scenesmith.py` removes furniture free joints, replaces the
generated convex decomposition with one static collision box per object, keeps
the textured visual meshes, and computes a collision-free robot spawn.
