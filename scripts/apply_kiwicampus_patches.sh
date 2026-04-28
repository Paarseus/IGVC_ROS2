#!/usr/bin/env bash
# Apply local patches to src/semantic_segmentation_layer/ (from kiwicampus).
# Idempotent: if a patch is already applied, that one is skipped.
#
# Run AFTER `vcs import src < avros.repos` and BEFORE `colcon build`.
# Remove this script (and the patches/ files) once upstream merges them.
#
# Patches applied (in order):
#   pr1 — Humble build fix (CMakeLists modernization, missing <deque>)
#   pr2 — temporal_tile_map_ mutex fix in updateBounds (data race that left
#         master grid empty 90% of cycles; see CHANGELOG_2026-04-25 §3)

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
PKG_DIR="$REPO_ROOT/src/semantic_segmentation_layer"
PATCH_DIR="$REPO_ROOT/src/avros_bringup/patches"

PATCHES=(
  "kiwicampus_pr1.patch"
  "kiwicampus_pr2_mutex.patch"
)

if [[ ! -d "$PKG_DIR" ]]; then
  echo "error: $PKG_DIR not found. Run 'vcs import src < avros.repos' first." >&2
  exit 1
fi

cd "$PKG_DIR"

for name in "${PATCHES[@]}"; do
  patch="$PATCH_DIR/$name"
  if [[ ! -f "$patch" ]]; then
    echo "error: patch file not found: $patch" >&2
    exit 1
  fi

  if git apply --reverse --check "$patch" >/dev/null 2>&1; then
    echo "$name: already applied (skipping)"
    continue
  fi

  echo "$name: applying via git am..."
  if ! git am --keep-non-patch "$patch"; then
    echo "error: git am failed for $name. Aborting and cleaning up." >&2
    git am --abort 2>/dev/null || true
    exit 1
  fi

  echo "$name: applied."
done
