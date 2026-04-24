#!/usr/bin/env bash
# Apply local patches to src/semantic_segmentation_layer/ (from kiwicampus).
# Idempotent: if the patch is already applied, exits 0 with a notice.
#
# Run AFTER `vcs import src < avros.repos` and BEFORE `colcon build`.
# Remove this script (and the patches/ file) once upstream merges PR #1.

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
PKG_DIR="$REPO_ROOT/src/semantic_segmentation_layer"
PATCH="$REPO_ROOT/src/avros_bringup/patches/kiwicampus_pr1.patch"

if [[ ! -d "$PKG_DIR" ]]; then
  echo "error: $PKG_DIR not found. Run 'vcs import src < avros.repos' first." >&2
  exit 1
fi

if [[ ! -f "$PATCH" ]]; then
  echo "error: patch file not found: $PATCH" >&2
  exit 1
fi

cd "$PKG_DIR"

# Detect whether the patch is already applied by reverse-applying --check.
# If reverse-apply succeeds, the patch is present; skip.
if git apply --reverse --check "$PATCH" >/dev/null 2>&1; then
  echo "kiwicampus_pr1.patch: already applied (skipping)"
  exit 0
fi

# Otherwise, apply via git am (patch file is mbox-format from GitHub pull/1.patch).
echo "kiwicampus_pr1.patch: applying via git am..."
if ! git am --keep-non-patch "$PATCH"; then
  echo "error: git am failed. Aborting and cleaning up." >&2
  git am --abort 2>/dev/null || true
  exit 1
fi

echo "kiwicampus_pr1.patch: applied."
