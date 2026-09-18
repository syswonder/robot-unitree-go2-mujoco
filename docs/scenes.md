# Scenes

`assets/environments/manifest.json` contains five mesh environments.
The default is `scenesmith_house_185`. Positions below use MuJoCo XY in meters;
all spawns stand on a floor at Z=0. House 186 faces -X (yaw pi radians), toward
the bedroom interior; the other scenes face +X (yaw 0). Robot base height is
applied by the runtime.

| ID | Genuine upstream content | Spawn XY | Clearance |
| --- | --- | --- | --- |
| `scenesmith_house_185` | House/scene_185: living room and bathroom | 4.88, 1.78 | 1.0775 m |
| `scenesmith_house_186` | House/scene_186: bedroom and bathroom | 4.48, 2.78 | 0.6225 m |
| `go2_rl_stairs` | go2_rl_gym stairs.xml, all 169 geometries | -0.5, 0 | 1.65 m |
| `go2_rl_track` | go2_rl_gym race_track.xml, all 236 geometries | -0.5, 0 | 0.98 m |
| `scenesmith_multilevel_house` | House 191 ground floor, House 188 upper floor, calibrated straight stair | -2.95, -1.70 | 0.50 m |

House 185 and House 186 remain distinct, uncombined single-floor exports. The
multilevel package is a separate composition imported from the adjacent
MuJoCo-GS-Web project: House 191 is the ground floor, House 188 is the upper
floor, and a 25-step flight provides the physical connection. Its source member
hashes and generation parameters are retained in that environment directory.

## Import

The following importer rebuilds the original two houses and two courses. Run
from the package root with Python 3.10+ and curl. The downloads total about
2.02 GB; the resulting runtime assets total about 213 MB. Allow 4 GB free space.

```sh
python3 scripts/download-scenes.py --cache /tmp/go2-scene-sources
# Same pinned files and SHA-256 checks through an alternate dataset host:
python3 scripts/download-scenes.py --cache /tmp/go2-scene-sources --endpoint https://hf-mirror.com
# Independently rebuild just the courses or houses:
python3 scripts/download-scenes.py --cache /tmp/go2-scene-sources --only courses
```

The importer verifies source checksums before conversion and reuses verified
archives. Each package's `provenance.json` records source path, revision, SHA-256,
license and converter. House revision is
`5425b6ac0ee7d5043af75f9ec78494cf5c94f3ca`; course revision is
`30e74dc507bec7a642a8c98be26081f2c6f0822d`. Runtime imports do not require the
dataset cache or any other local robot package.

## Geometry

The converter is derived from mujoco_robonix/scripts/prepare-scenesmith.py.
It preserves textured OBJ faces and UVs, deduplicates identical assets and freezes
furniture using conservative object bounding boxes. House 185 retains 63 mesh
definitions and 55 image textures; house 186 retains 60 and 52. Original room
colliders remain, and guards seal exposed floor boundaries without sealing the
shared room boundary. Furniture box collision is less detailed than the source's
convex decomposition and may block space under furniture.

House floor bounds and shared door openings in world XY (meters), measured from
the room box colliders:

| House | Main-room floor X; Y | Bathroom floor X; Y | Shared door center | Door opening Y |
| --- | --- | --- | --- | --- |
| 185 | [0, 6]; [0, 4.5] | [6, 8.2]; [1.05, 3.45] | (6, 2.03744) | [1.58744, 2.48744] |
| 186 | [0, 5.2]; [0, 4.2] | [5.2, 7.6]; [0.8, 3.4] | (5.2, 1.99670) | [1.54670, 2.44670] |

Both shared openings are 0.90 m wide, with wall thickness spanning 0.05 m on
each side of the shared X boundary. A planner using a 0.50 m circular inflation
radius cannot pass these openings; that radius reserves spawn space, whereas
door traversal requires the actual oriented robot footprint and appropriate
clearance. House 185's goal (2.04, 0.65) lies inside the living-room floor, away
from the shared doorway, but intersects `living_room_sofa_0_static_collision`:
the point (2.04, 0.65, 0.30) is inside the actual oriented box, not merely its
world-aligned bounds. This goal is occupied. Floor bounds alone do not establish
a collision-free route.

Course imports preserve every upstream geometry's size, position, quaternion,
material and contact parameters. The robot include is removed; each original geom
becomes a visible noncolliding copy in group 1 and an identical static collider in
group 3. The source courses use MJCF boxes and procedural textures, rendered by
the mesh renderer. There are no substitute ramps or reconstructed course layouts.
All house world colliders also use group 3, leaving group 4 for the robot.

Spawn selection reserves a 0.50 m radius disk, containing a conservative
0.84 x 0.44 m Go2 footprint. The house search checks floor support, walls and
furniture; it fails if no safe position exists. This validates initial placement,
not successful locomotion across every upstream obstacle.
The importer explicitly authors house 186's yaw as `3.141592653589793` so its
forward camera initially faces the interior instead of the nearby east wall.
The heading is recorded in spawn.json, generation_report.json and provenance.json;
direct converter invocations can reproduce it with `--spawn-yaw 3.141592653589793`.

## Validation

```sh
uv venv /tmp/go2-scene-validation
uv pip install --python /tmp/go2-scene-validation/bin/python mujoco==3.3.7
/tmp/go2-scene-validation/bin/python scripts/download-scenes-validate.py --cache /tmp/go2-scene-sources
```

The check loads the original four XMLs, verifies their IDs and indexed files, asserts
static colliders use group 3, probes 35 floor support points across the footprint,
and checks a 0.84 x 0.44 x 0.78 m volume above the floor has zero contacts.
`--cache` additionally compares every course visual and collider against upstream
XML geometry and checks that texture/material definitions remain unchanged.
Those four imports passed these native MuJoCo 3.3.7 checks and loaded successfully
in the package's mujoco-js WASM engine (MuJoCo 3.3.8).

The multilevel package is checked with
`sim/tests/native_smoke.py --environment scenesmith_multilevel_house --policy`;
its scene-level stair annotation is `multifloor.yaml`, and its per-floor Mapping
artifacts are under `assets/maps/`. The complete sensor-built artifacts are
`scenesmith_multilevel_floor_1_v2` (439 RTAB-Map nodes, 288 x 147 occupancy)
and `scenesmith_multilevel_floor_2_v2` (787 nodes, 241 x 153 occupancy).

## Licenses

[SceneSmith example scenes](https://huggingface.co/datasets/nepfaff/scenesmith-example-scenes)
declare Apache-2.0; the complete terms are in
`assets/environments/LICENSE.scenesmith`, with attribution in each house's NOTICE.md.
[go2_rl_gym](https://github.com/wty-yy/go2_rl_gym) uses MIT terms and retains a
Unitree BSD-3-Clause notice; the complete upstream file is included as
`LICENSE.upstream` in each course package. Dataset licensing is separate from
the SceneSmith generator project's MIT license.
