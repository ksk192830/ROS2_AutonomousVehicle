#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
ROS_SETUP="/opt/ros/humble/setup.bash"
WORKSPACE_SETUP="$WORKSPACE_DIR/install/setup.bash"
RUNTIME_DIR="${XDG_RUNTIME_DIR:-/tmp}/e2e-exhibition-${UID}"
STATE_DIR="${XDG_STATE_HOME:-$HOME/.local/state}/e2e-exhibition"
PID_FILE="$RUNTIME_DIR/general_sim.pid"
LOCK_FILE="$RUNTIME_DIR/launcher.lock"

mkdir -p "$RUNTIME_DIR" "$STATE_DIR"
exec 9>"$LOCK_FILE"
if ! flock -n 9; then
  exit 0
fi

notify() {
  local title="$1"
  local message="$2"
  if command -v notify-send >/dev/null 2>&1; then
    notify-send "$title" "$message" || true
  fi
}

show_error() {
  local message="$1"
  printf '%s\n' "$message" >&2
  notify "자율주행 탐험" "$message"
  if command -v zenity >/dev/null 2>&1; then
    zenity --error --title="자율주행 탐험" --text="$message" || true
  fi
}

stop_previous_run() {
  [[ -f "$PID_FILE" ]] || return 0
  local old_pid
  old_pid="$(tr -cd '0-9' < "$PID_FILE")"
  if [[ -z "$old_pid" ]] || ! kill -0 -- "-$old_pid" 2>/dev/null; then
    rm -f "$PID_FILE"
    return 0
  fi

  local command_line
  command_line="$(ps -p "$old_pid" -o command= 2>/dev/null || true)"
  if [[ -z "$command_line" ]] && ps -eo pgid,args | awk -v group="$old_pid" \
      '$1 == group && /gzserver \/tmp\/track_exhibition_/ { found=1 } END { exit !found }'; then
    # The launcher can exit before its simulator. The saved process group
    # still belongs to this run; finish cleaning it before opening another.
    :
  elif [[ "$command_line" != *"ros2 launch launch_pkg general_sim.py"* \
        && "$command_line" != *"ros2 launch launch_pkg race_game.py"* ]]; then
    rm -f "$PID_FILE"
    return 0
  fi

  notify "자율주행 탐험" "기존 프로그램을 정리하고 체험을 시작합니다."
  kill -INT -- "-$old_pid" 2>/dev/null || kill -INT "$old_pid" 2>/dev/null || true
  for _ in $(seq 1 32); do
    kill -0 -- "-$old_pid" 2>/dev/null || break
    sleep 0.25
  done
  if kill -0 -- "-$old_pid" 2>/dev/null; then
    kill -TERM -- "-$old_pid" 2>/dev/null || kill -TERM "$old_pid" 2>/dev/null || true
    for _ in $(seq 1 16); do
      kill -0 -- "-$old_pid" 2>/dev/null || break
      sleep 0.25
    done
  fi
  if kill -0 -- "-$old_pid" 2>/dev/null; then
    kill -KILL -- "-$old_pid" 2>/dev/null || kill -KILL "$old_pid" 2>/dev/null || true
  fi
  rm -f "$PID_FILE"
}

if [[ ! -f "$ROS_SETUP" || ! -f "$WORKSPACE_SETUP" ]]; then
  show_error "프로젝트 설치가 완료되지 않았습니다. install.sh를 먼저 실행해 주세요."
  exit 1
fi
if [[ -z "${DISPLAY:-}" ]]; then
  show_error "화면 연결을 찾지 못했습니다. 모니터가 연결된 데스크톱에서 실행해 주세요."
  exit 1
fi

stop_previous_run

# Give this launch its own Gazebo transport master. An unrelated simulator
# may already own the default port; never stop it to start this exhibition.
# Preserve an explicitly configured master address.
if [[ -z "${GAZEBO_MASTER_URI:-}" ]]; then
  exhibition_gazebo_port="$(/usr/bin/python3 - <<'PY'
import socket
with socket.socket() as listener:
    listener.bind(('127.0.0.1', 0))
    print(listener.getsockname()[1])
PY
)"
  export GAZEBO_MASTER_URI="http://127.0.0.1:${exhibition_gazebo_port}"
fi

set +u
source "$ROS_SETUP"
source "$WORKSPACE_SETUP"
set -u

export E2E_COMPACT_LOGS="${E2E_COMPACT_LOGS:-1}"
export E2E_PERFORMANCE_MODE="${E2E_PERFORMANCE_MODE:-1}"
export E2E_FRONT_CAMERA_RATE="${E2E_FRONT_CAMERA_RATE:-120}"
# The launch runtime checks driver health before choosing GPU or CPU graphics.

timestamp="$(date +%Y%m%d-%H%M%S)"
LOG_FILE="$STATE_DIR/race-game-$timestamp.log"
ln -sfn "$LOG_FILE" "$STATE_DIR/latest.log"

notify "자율주행 탐험" "체험을 시작합니다. 차량 준비에 잠시 시간이 걸릴 수 있습니다."
cd "$WORKSPACE_DIR"
nohup setsid ros2 launch launch_pkg race_game.py \
  >"$LOG_FILE" 2>&1 </dev/null 9>&- &
launcher_pid=$!
printf '%s\n' "$launcher_pid" > "$PID_FILE"

sleep 2
if ! kill -0 "$launcher_pid" 2>/dev/null; then
  rm -f "$PID_FILE"
  show_error "게임을 시작하지 못했습니다. 로그: $STATE_DIR/latest.log"
  exit 1
fi

notify "자율주행 탐험" "체험 화면과 차량을 초기화하고 있습니다."
