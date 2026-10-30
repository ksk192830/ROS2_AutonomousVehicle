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
  notify "E2E 자율주행" "$message"
  if command -v zenity >/dev/null 2>&1; then
    zenity --error --title="E2E 자율주행" --text="$message" || true
  fi
}

stop_previous_run() {
  [[ -f "$PID_FILE" ]] || return 0
  local old_pid
  old_pid="$(tr -cd '0-9' < "$PID_FILE")"
  if [[ -z "$old_pid" ]] || ! kill -0 "$old_pid" 2>/dev/null; then
    rm -f "$PID_FILE"
    return 0
  fi

  local command_line
  command_line="$(ps -p "$old_pid" -o command= 2>/dev/null || true)"
  if [[ "$command_line" != *"ros2 launch launch_pkg general_sim.py"* \
        && "$command_line" != *"ros2 launch launch_pkg race_game.py"* ]]; then
    rm -f "$PID_FILE"
    return 0
  fi

  notify "E2E 자율주행" "기존 전시 프로그램을 정리하고 다시 시작합니다."
  kill -INT -- "-$old_pid" 2>/dev/null || kill -INT "$old_pid" 2>/dev/null || true
  for _ in $(seq 1 32); do
    kill -0 "$old_pid" 2>/dev/null || break
    sleep 0.25
  done
  if kill -0 "$old_pid" 2>/dev/null; then
    kill -TERM -- "-$old_pid" 2>/dev/null || kill -TERM "$old_pid" 2>/dev/null || true
    for _ in $(seq 1 16); do
      kill -0 "$old_pid" 2>/dev/null || break
      sleep 0.25
    done
  fi
  if kill -0 "$old_pid" 2>/dev/null; then
    kill -KILL -- "-$old_pid" 2>/dev/null || kill -KILL "$old_pid" 2>/dev/null || true
  fi
  rm -f "$PID_FILE"
}

if [[ ! -f "$ROS_SETUP" ]]; then
  show_error "ROS 2 Humble이 없습니다. 프로젝트 폴더에서 install.sh를 먼저 실행해 주세요."
  exit 1
fi
if [[ ! -f "$WORKSPACE_SETUP" ]]; then
  show_error "프로젝트가 아직 빌드되지 않았습니다. install.sh를 먼저 실행해 주세요."
  exit 1
fi
if [[ -z "${DISPLAY:-}" ]]; then
  show_error "화면 연결을 찾지 못했습니다. 모니터가 연결된 데스크톱에서 실행해 주세요."
  exit 1
fi

stop_previous_run

set +u
source "$ROS_SETUP"
source "$WORKSPACE_SETUP"
set -u

export E2E_COMPACT_LOGS="${E2E_COMPACT_LOGS:-1}"
export E2E_PERFORMANCE_MODE="${E2E_PERFORMANCE_MODE:-1}"
# The launch runtime checks driver health before choosing GPU or CPU graphics.

timestamp="$(date +%Y%m%d-%H%M%S)"
LOG_FILE="$STATE_DIR/general_sim-$timestamp.log"
ln -sfn "$LOG_FILE" "$STATE_DIR/latest.log"

notify "E2E 자율주행" "전시 인터페이스를 시작합니다. 초기화에 잠시 시간이 걸릴 수 있습니다."
cd "$WORKSPACE_DIR"
nohup setsid ros2 launch launch_pkg general_sim.py \
  >"$LOG_FILE" 2>&1 </dev/null 9>&- &
launcher_pid=$!
printf '%s\n' "$launcher_pid" > "$PID_FILE"

sleep 2
if ! kill -0 "$launcher_pid" 2>/dev/null; then
  rm -f "$PID_FILE"
  show_error "전시 프로그램을 시작하지 못했습니다. 로그: $STATE_DIR/latest.log"
  exit 1
fi

notify "E2E 자율주행" "전시 화면과 차량을 초기화하고 있습니다."
