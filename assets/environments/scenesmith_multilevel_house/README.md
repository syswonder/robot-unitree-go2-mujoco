# Complete Two-Story SceneSmith House

This self-contained copy is registered by the surrounding
`robot-unitree-go2_mujoco` deployment. Robonix floor annotations are in
`multifloor.yaml`; prebuilt per-floor maps are stored in the deployment's
`assets/maps` directory.

Ground floor: newly imported House/scene_191.tar. Upper floor: newly imported House/scene_188.tar. All source room and furniture bodies are retained. Entrance openings and explicitly listed furniture relocations are the only layout changes.

A separate south-side stair hall connects the floors with one straight flight: 25 steps, 0.12 m rise, 0.30 m tread, 1.20 m width, 3.0 m floor height. The upper gallery is beside the flight, not above it. Each room and the stairs have fill lights.

Textures are limited to 512 pixels by default to keep the complete merged model below the browser MuJoCo WASM memory limit. Source textures remain unchanged in the cache.

Visual meshes use a UV/normal-aware meshoptimizer LOD (18% target face count, 0.5% relative error limit). Original prepared meshes and collision boxes are unchanged. Per-mesh reductions are recorded in generation_report.json.

## Start

The original source project can run the scene manually. In this deployment use:

```sh
bash sim/start.sh --backend native --headless --environment scenesmith_multilevel_house
```

The `floor_transition` skill owns policy switching and centerline traversal for
the calibrated stair. The policy's built-in test-course route is not used.

The deployment copy uses two low-intensity directional lights instead of the
source package's room-local point lights because the native runtime only accepts
directional scene lights. Stair materials have no self-emission, so the steps
retain visible contrast instead of clipping to white in the native viewer.

## Rebuild

```sh
bash scripts/prepare-multilevel-sources.sh
npm run import:scenesmith-house
node --test tests/multilevel-house.test.mjs tests/environment-packages.test.mjs
```

Source preparation reuses ../robot-unitree-go2_mujoco/scripts/prepare-scenesmith.py. Override SCENESMITH_CONVERTER if that installed converter is elsewhere. SCENE_PYTHON defaults to .venv-scene-geometry/bin/python; python3 needs requests. The offline LOD script needs the project's meshoptimizer@1.2.0 dev dependency (npm install). No system configuration is changed. The downloader uses hf-mirror.com with a pinned Hugging Face revision and only fetches referenced MuJoCo visual resources.

Entrance coordinates, floor offsets and furniture relocations are configured in scripts/config/scenesmith-multilevel.json. Cached raw/prepared sources are in .cache/scenesmith/; raw, prepared and simplified packages are kept separately. Runtime assets do not depend on that cache. Successful rebuilds preserve the previous runtime package in a printed /tmp/scenesmith-multilevel-backup-* directory. generation_report.json records source identities, member hashes, room/body counts and geometry parameters.
