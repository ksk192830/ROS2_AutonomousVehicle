#!/usr/bin/env bash
# Portable static workspace checks (Mac Bash 3.x / Ubuntu). Does not build ROS.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
projects=(3rd_skku_contest 4th_skku_contest gyeonggido_contest autonomous_simulation)
issues=0

echo "ROS2_AutonomousVehicle workspace check"
for project in "${projects[@]}"; do
  src="$ROOT/projects/$project/src"
  if [[ ! -d "$src" ]]; then
    printf '[PENDING] %s: source not migrated\n' "$project"
    continue
  fi

  pkgs="$(find "$src" -type f -name package.xml -print)"
  if [[ -z "$pkgs" ]]; then
    printf '[WARN] %s: no package.xml\n' "$project"
    issues=$((issues + 1))
    continue
  fi
  count="$(printf '%s\n' "$pkgs" | wc -l | tr -d ' ')"
  names="$(printf '%s\n' "$pkgs" | while IFS= read -r f; do sed -n 's/.*<name>\([^<]*\)<\/name>.*/\1/p' "$f" | head -n 1; done)"
  duplicate="$(printf '%s\n' "$names" | sort | uniq -d)"
  if [[ -n "$duplicate" ]]; then
    printf '[WARN] %s: duplicate packages: %s\n' "$project" "$duplicate"
    issues=$((issues + 1))
  else
    printf '[FOUND] %s: %s ROS 2 packages\n' "$project" "$count"
  fi
done

model="$ROOT/projects/3rd_skku_contest/src/camera_pkg/camera_pkg/model/final.pt"
if [[ ! -f "$model" ]]; then
  echo "[PENDING] 3rd SKKU: final.pt model has not been added"
elif grep -aq 'https://git-lfs.github.com/spec/v1' "$model"; then
  echo "[WARN] 3rd SKKU: final.pt is only an LFS pointer; run git lfs pull"
  issues=$((issues + 1))
else
  echo "[FOUND] 3rd SKKU: local final.pt present"
fi
echo "KSAE 2025, ACK 2026 and KSAE 2026 papers: PDF documentation, not ROS workspaces"
echo "Static checks only; ROS 2 build, model inference and hardware runtime remain unverified."
if (( issues > 0 )); then exit 1; fi
