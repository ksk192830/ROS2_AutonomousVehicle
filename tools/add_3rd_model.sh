#!/usr/bin/env bash
# Install a locally authorized .pt model for runtime use only; do not publish.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ "$#" -ne 1 || ! -f "$1" ]]; then
  echo "Usage: bash tools/add_3rd_model.sh /path/to/final.pt" >&2
  exit 2
fi
dst="$ROOT/projects/3rd_skku_contest/src/camera_pkg/camera_pkg/model/final.pt"
if [[ -e "$dst" ]]; then
  echo "Destination already exists: $dst (not overwriting)" >&2
  exit 1
fi
mkdir -p "$(dirname "$dst")"
cp "$1" "$dst"
echo "[OK] Installed locally: $dst"
echo "[INFO] AI weights are Git-ignored. Do not force-add or redistribute."
