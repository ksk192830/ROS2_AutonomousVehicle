#!/usr/bin/env bash
# Run on Ubuntu 22.04 with ROS 2 Humble installed, one workspace at a time.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PROJECT="${1:-}"
case "$PROJECT" in 4th_skku_contest|autonomous_simulation) ;; *) echo 'Usage: bash tools/build_workspace.sh {4th_skku_contest|autonomous_simulation}' >&2; exit 2;; esac
[[ -f /etc/os-release ]] || { echo 'Ubuntu 22.04 required' >&2; exit 1; }
source /etc/os-release
[[ "$ID" == ubuntu && "$VERSION_ID" == 22.04 ]] || { echo 'Ubuntu 22.04 required' >&2; exit 1; }
[[ -f /opt/ros/humble/setup.bash ]] || { echo 'Install ROS 2 Humble first; see docs/reproduction.md' >&2; exit 1; }
# Reject an already sourced competing workspace rather than silently mixing packages.
if [[ "${AMENT_PREFIX_PATH:-}" == *"/projects/"* ]]; then
  echo 'Open a fresh terminal without another project overlay before building.' >&2
  exit 1
fi
set +u
source /opt/ros/humble/setup.bash
set -u
cd "$ROOT/projects/$PROJECT"
rosdep install --from-paths src --ignore-src --rosdistro humble -r -y
colcon build --symlink-install --event-handlers console_direct+
echo "Build finished. In this terminal run: source $ROOT/projects/$PROJECT/install/setup.bash"
echo 'Build success does not validate runtime assets or driving behavior.'
