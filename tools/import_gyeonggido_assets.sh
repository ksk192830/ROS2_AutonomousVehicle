#!/usr/bin/env bash
# Import three trained .pt weights and a parking-map PNG from a source Git repo.
# Binary contents are verified against their source Git blob IDs.
# Does not change source repository or auto-commit/push any files.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SOURCE="${1:-}"
if [[ ! "$SOURCE" =~ ^[A-Za-z0-9_.-]+/[A-Za-z0-9_.-]+$ ]]; then
  echo "Usage: bash tools/import_gyeonggido_assets.sh OWNER/REPOSITORY" >&2
  exit 2
fi

for binary in git gh; do
  command -v "$binary" >/dev/null 2>&1 || {
    echo "Missing required command: $binary" >&2
    exit 1
  }
done
git lfs version >/dev/null 2>&1 || {
  echo "Git LFS is required (macOS: brew install git-lfs)" >&2
  exit 1
}
gh auth status >/dev/null 2>&1 || {
  echo "Log in first: gh auth login" >&2
  exit 1
}
gh auth setup-git >/dev/null
git -C "$ROOT" lfs install --local >/dev/null

SCRATCH="$(mktemp -d "${TMPDIR:-/tmp}/av_gyeonggi.XXXXXX")"
trap 'rm -rf "$SCRATCH"' EXIT
git clone --quiet --depth 1 --filter=blob:none --no-checkout \
  "https://github.com/${SOURCE}.git" "$SCRATCH/source"

FILES=(
  "src/camera_pkg/camera_pkg/model/track.pt"
  "src/camera_pkg/camera_pkg/model/parking_front.pt"
  "src/camera_pkg/camera_pkg/model/parking_rear.pt"
  "convenient/parking_map/map_image.png"
)
count=0
for path in "${FILES[@]}"; do
  expected="$(git -C "$SCRATCH/source" rev-parse "HEAD:$path")"
  [[ "$expected" =~ ^[0-9a-f]{40}$ ]] || {
    echo "[STOP] Invalid source blob ID for $path" >&2
    exit 1
  }
  target="$ROOT/projects/gyeonggido_contest/$path"
  if [[ -f "$target" ]]; then
    current="$(git -C "$ROOT" hash-object --no-filters "$target")"
    if [[ "$current" == "$expected" ]]; then
      echo "[SKIP] $path already matches"
      continue
    fi
    echo "[STOP] Existing file differs: $path (will not overwrite)" >&2
    exit 1
  fi
  tmpfile="$SCRATCH/$(basename "$path")"
  git -C "$SCRATCH/source" show "HEAD:$path" > "$tmpfile"
  actual="$(git -C "$ROOT" hash-object --no-filters "$tmpfile")"
  bytes="$(wc -c < "$tmpfile" | tr -d ' ')"
  if [[ "$actual" != "$expected" || "$bytes" -lt 3000 ]]; then
    echo "[STOP] Git SHA or size check failed: $path" >&2
    exit 1
  fi
  mkdir -p "$(dirname "$target")"
  mv "$tmpfile" "$target"
  echo "[OK] $path ($bytes bytes)"
  count=$((count + 1))
done

echo "[OK] Imported $count asset(s). Nothing was committed or pushed."
echo "[INFO] Model weights are local-only and Git-ignored. Do not force-add, commit, or redistribute."\n