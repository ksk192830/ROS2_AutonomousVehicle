#!/usr/bin/env bash
# Import runtime model weights from a private source repository.
# Git's partial clone fetches only requested .pt blobs, not datasets.
# No source repository address is written to the public documentation.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SOURCE="${1:-}"
if [[ ! "$SOURCE" =~ ^[A-Za-z0-9_.-]+/[A-Za-z0-9_.-]+$ ]]; then
  echo "Usage: bash tools/import_4th_runtime_models.sh OWNER/REPOSITORY" >&2
  exit 2
fi

for binary in gh git; do
  command -v "$binary" >/dev/null 2>&1 || {
    echo "Required command is missing: $binary" >&2
    exit 1
  }
done

git lfs version >/dev/null 2>&1 || {
  echo "Git LFS is required (macOS: brew install git-lfs)" >&2
  exit 1
}
gh auth status >/dev/null 2>&1 || {
  echo "GitHub CLI authentication required: gh auth login" >&2
  exit 1
}
gh auth setup-git >/dev/null
git -C "$ROOT" lfs install --local >/dev/null

SCRATCH="$(mktemp -d "${TMPDIR:-/tmp}/av4_models.XXXXXX")"
trap 'rm -rf "$SCRATCH"' EXIT
echo "[FETCH] Creating a minimal private-source Git clone"
git clone --quiet --depth 1 --filter=blob:none --no-checkout \
  "https://github.com/${SOURCE}.git" "$SCRATCH/source"

TARGET_DIR="$ROOT/projects/4th_skku_contest/src/camera_pkg/camera_pkg/model"
mkdir -p "$TARGET_DIR"

NAMES=(crosswalk.pt parking_front.pt parking_rear.pt track.pt traffic_light_sim.pt)
count=0
for name in "${NAMES[@]}"; do
  source_path="src/camera_pkg/camera_pkg/model/$name"
  expected="$(git -C "$SCRATCH/source" rev-parse "HEAD:$source_path")"
  if [[ ! "$expected" =~ ^[0-9a-f]{40}$ ]]; then
    echo "[STOP] Invalid Git object ID for $name" >&2
    exit 1
  fi

  target="$TARGET_DIR/$name"
  if [[ -e "$target" ]]; then
    current="$(git -C "$ROOT" hash-object --no-filters "$target")"
    if [[ "$current" == "$expected" ]]; then
      echo "[SKIP] $name already matches"
      continue
    fi
    echo "[STOP] Existing model differs: $name (will not overwrite)" >&2
    exit 1
  fi

  incoming="$SCRATCH/$name"
  git -C "$SCRATCH/source" show "HEAD:$source_path" > "$incoming"
  actual="$(git -C "$ROOT" hash-object --no-filters "$incoming")"
  size="$(wc -c < "$incoming" | tr -d ' ')"
  if [[ "$actual" != "$expected" || "$size" -lt 100000 ]]; then
    echo "[STOP] Unexpected model contents or Git SHA mismatch for $name" >&2
    exit 1
  fi
  mv "$incoming" "$target"
  echo "[OK] $name ($size bytes)"
  count=$((count + 1))
done

echo "[OK] Imported $count model(s). No files have been committed or pushed."
echo "[INFO] Model weights are local-only and Git-ignored. Do not force-add, commit, or redistribute."\n