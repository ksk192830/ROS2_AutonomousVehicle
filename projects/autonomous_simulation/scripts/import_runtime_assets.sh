#!/usr/bin/env bash
# Recover binary models and trained weights from an authorized source checkout.
# Leaves the original repository unchanged; never stages, commits, or pushes.
# The source location is an argument and is never recorded in tracked files.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SOURCE="${1:-}"
MODE="${2:-core}"
if [[ ! "$SOURCE" =~ ^[A-Za-z0-9_.-]+/[A-Za-z0-9_.-]+$ ]]; then
  echo "Usage: bash scripts/import_runtime_assets.sh OWNER/REPOSITORY [core|expo|all]" >&2
  exit 2
fi
case "$MODE" in core|expo|all) ;; *) echo "Unknown mode: $MODE" >&2; exit 2;; esac

for cmd in git gh; do
  command -v "$cmd" >/dev/null 2>&1 || { echo "Missing $cmd" >&2; exit 1; }
done
git lfs version >/dev/null 2>&1 || { echo "Install git-lfs first" >&2; exit 1; }
gh auth status >/dev/null 2>&1 || { echo "Run gh auth login first" >&2; exit 1; }
gh auth setup-git >/dev/null
git -C "$ROOT" lfs install --local >/dev/null
TMP="$(mktemp -d "${TMPDIR:-/tmp}/autonomous_simulation.XXXXXX")"
trap 'rm -rf "$TMP"' EXIT
git clone --quiet --branch expo --depth 1 --filter=blob:none --no-checkout \
  "https://github.com/$SOURCE.git" "$TMP/source"

is_binary_asset() {
  case "$1" in
    *.png|*.jpg|*.jpeg|*.obj|*.dae|*.fbx|*.stl|*.3ds|*.c4d|*.pt|*.pth) return 0 ;;
    *) return 1 ;;
  esac
}
include_path() {
  local path="$1"
  case "$path" in
    src/camera_pkg/camera_pkg/model/*.pt|src/camera_pkg/camera_pkg/model/*.pth)
      return 0 ;;
    src/simulation_pkg/simulation_pkg/data/sim.pt)
      [[ "$MODE" == all ]]; return ;;
    src/simulation_pkg/models/*)
      is_binary_asset "$path" || return 1
      if [[ "$MODE" == core ]]; then
        case "$path" in
          src/simulation_pkg/models/mercedes_split_tex/*|src/simulation_pkg/models/mercedes_obstacle/*|src/simulation_pkg/models/race_track/*|src/simulation_pkg/models/traffic_sim/*)
            return 0 ;;
          *) return 1 ;;
        esac
      fi
      return 0 ;;
    *) return 1 ;;
  esac
}
sha256_file() {
  if command -v shasum >/dev/null 2>&1; then
    shasum -a 256 "$1" | awk '{print $1}'
  else
    sha256sum "$1" | awk '{print $1}'
  fi
}

count=0
while IFS= read -r relative; do
  include_path "$relative" || continue
  destination="$ROOT/$relative"
  source_sha="$(git -C "$TMP/source" rev-parse "HEAD:$relative")"
  [[ "$source_sha" =~ ^[a-f0-9]{40}$ ]] || { echo "[STOP] Invalid SHA $relative" >&2; exit 1; }

  pointer="$TMP/incoming"
  git -C "$TMP/source" show "HEAD:$relative" > "$pointer"
  if grep -aq '^version https://git-lfs.github.com/spec/v1' "$pointer"; then
    expected_oid="$(sed -n 's/^oid sha256://p' "$pointer" | head -n 1)"
    expected_size="$(sed -n 's/^size //p' "$pointer" | head -n 1)"
    [[ "$expected_oid" =~ ^[a-f0-9]{64}$ ]] || { echo "[STOP] Invalid LFS pointer $relative" >&2; exit 1; }
    git -C "$TMP/source" lfs smudge < "$pointer" > "$TMP/resolved"
    actual_oid="$(sha256_file "$TMP/resolved")"
    actual_size="$(wc -c < "$TMP/resolved" | tr -d ' ')"
    [[ "$actual_oid" == "$expected_oid" && "$actual_size" == "$expected_size" ]] || {
      echo "[STOP] LFS SHA256 or size mismatch: $relative" >&2; exit 1;
    }
    mv "$TMP/resolved" "$pointer"
  else
    actual_sha="$(git -C "$ROOT" hash-object --no-filters "$pointer")"
    [[ "$actual_sha" == "$source_sha" ]] || { echo "[STOP] Git blob mismatch: $relative" >&2; exit 1; }
  fi

  if [[ -f "$destination" ]]; then
    if cmp -s "$destination" "$pointer"; then
      echo "[SKIP] $relative"
      continue
    fi
    echo "[STOP] Local file differs, refusing overwrite: $relative" >&2
    exit 1
  fi
  mkdir -p "$(dirname "$destination")"
  mv "$pointer" "$destination"
  echo "[COPIED] $relative"
  count=$((count + 1))
done < <(git -C "$TMP/source" ls-tree -r --name-only HEAD)

echo "[OK] $count new binary assets copied locally. No files staged or pushed."
echo "Inspect third-party 3D-model permissions before publishing assets to GitHub."
