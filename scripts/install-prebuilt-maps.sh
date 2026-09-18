#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
SOURCE="$ROOT/assets/maps"
DESTINATION="${MAPPING_MAPS_DIR:-$HOME/.robonix/maps/mujoco-go2}"

command -v xz >/dev/null 2>&1 || {
  echo "xz is required to install the packaged maps." >&2
  exit 1
}

for map_id in scenesmith_multilevel_floor_1_v2 scenesmith_multilevel_floor_2_v2; do
  source_map="$SOURCE/$map_id"
  source_database="$source_map/rtabmap.db.xz"
  target_map="$DESTINATION/$map_id"
  [[ -f "$source_database" && -f "$source_map/meta.yaml" ]] || {
    echo "Missing packaged map artifact: $source_map" >&2
    exit 1
  }
  if [[ -e "$target_map" ]]; then
    [[ -f "$target_map/rtabmap.db" && -f "$target_map/meta.yaml" ]] || {
      echo "Refusing to overwrite incomplete map directory: $target_map" >&2
      exit 1
    }
    echo "[maps] keeping existing $map_id"
    continue
  fi
  mkdir -p "$DESTINATION"
  temporary="$DESTINATION/.${map_id}.tmp.$$"
  trap 'rm -rf -- "${temporary:-}"' EXIT
  mkdir "$temporary"
  cp -a "$source_map/." "$temporary/"
  xz --decompress --stdout "$source_database" > "$temporary/rtabmap.db"
  rm "$temporary/rtabmap.db.xz"
  mv "$temporary" "$target_map"
  trap - EXIT
  echo "[maps] installed $map_id"
done
