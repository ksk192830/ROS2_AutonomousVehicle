#!/usr/bin/env bash

source /opt/ros/humble/setup.bash
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
if [[ -f "$PROJECT_ROOT/install/setup.bash" ]]; then
    source "$PROJECT_ROOT/install/setup.bash"
else
    echo "Missing workspace setup: $PROJECT_ROOT/install/setup.bash" >&2
    exit 1
fi
set -u
export ROS_DOMAIN_ID=17

scenario="${1:-}"
state_file="${XDG_RUNTIME_DIR:-/tmp}/skku_4th/terminal-logs-off"
log_dir="$PROJECT_ROOT/log/terminal"
log_file="$log_dir/$scenario-$(date +%Y%m%d_%H%M%S).log"

case "$scenario" in
    general)
        title="일반주행"; banner="GENERAL"; color='\033[1;96m'
        command=(ros2 launch launch_pkg general.py)
        ;;
    mission)
        title="미션주행"; banner="MISSION"; color='\033[1;95m'
        command=(ros2 launch launch_pkg mission.py)
        ;;
    parking)
        title="주차"; banner="PARKING"; color='\033[1;93m'
        command=(ros2 launch launch_pkg parking.py)
        ;;
    control_p)
        title="control_p"; command=(ros2 run control_pkg control_p)
        ;;
    check_devices)
        title="포트 확인"; command=(python3 "$PROJECT_ROOT/convenient/check_devices.py")
        ;;
    *) exit 2 ;;
esac

terminal_window_id="${WINDOWID:-}"
if [[ "$scenario" == control_p ]] && command -v wmctrl >/dev/null 2>&1; then
    (
        # 패널을 제외한 실제 작업 영역의 맨 아래에 창을 맞춘다.
        workarea="$(xprop -root _NET_WORKAREA 2>/dev/null | sed 's/.*=//; s/,//g')"
        read -r work_x work_y work_width work_height _ <<<"$workarea"
        work_x="${work_x:-0}"
        work_y="${work_y:-0}"
        if [[ -z "${work_height:-}" ]]; then
            work_height="$(xrandr --current 2>/dev/null | awk '/\*/ {split($1, size, "x"); print size[2]; exit}')"
        fi
        work_height="${work_height:-1080}"
        target_y=$((work_y + work_height - 490))
        ((target_y < 0)) && target_y=0

        # Do not use an inherited WINDOWID here: when launched from VS Code it
        # can point to the VS Code window instead of the new Terminator window.
        # Match both the Terminator window class and the exact control_p title.
        for _ in {1..20}; do
            sleep 0.1
            control_window_id="$(
                wmctrl -lx 2>/dev/null | awk '
                    tolower($3) ~ /terminator/ && $0 ~ / control_p$/ {
                        print $1
                        exit
                    }
                '
            )"
            if [[ -n "$control_window_id" ]] && \
                    wmctrl -i -r "$control_window_id" -b remove,maximized_vert,maximized_horz 2>/dev/null && \
                    wmctrl -i -r "$control_window_id" -e "0,$work_x,$target_y,690,490" 2>/dev/null; then
                exit
            fi
        done
    ) &
elif [[ "$scenario" == general || "$scenario" == mission || "$scenario" == parking ]] && \
        command -v wmctrl >/dev/null 2>&1; then
    (
        for _ in {1..80}; do
            sleep 0.25
            if wmctrl -lx | grep -qi rviz; then
                sleep 0.5
                [[ -n "$terminal_window_id" ]] && wmctrl -i -a "$terminal_window_id"
                exit
            fi
        done
    ) &
fi

show_banner() {
    printf '\033[2J\033[H%b' "$color"
    case "$banner" in
        GENERAL)
            printf '%s\n' \
                '  ██████  ███████ ███    ██ ███████ ██████   █████  ██      ' \
                ' ██       ██      ████   ██ ██      ██   ██ ██   ██ ██      ' \
                ' ██   ███ █████   ██ ██  ██ █████   ██████  ███████ ██      ' \
                ' ██    ██ ██      ██  ██ ██ ██      ██   ██ ██   ██ ██      ' \
                '  ██████  ███████ ██   ████ ███████ ██   ██ ██   ██ ███████ '
            ;;
        MISSION)
            printf '%s\n' \
                ' ███    ███ ██ ███████ ███████ ██  ██████  ███    ██ ' \
                ' ████  ████ ██ ██      ██      ██ ██    ██ ████   ██ ' \
                ' ██ ████ ██ ██ ███████ ███████ ██ ██    ██ ██ ██  ██ ' \
                ' ██  ██  ██ ██      ██      ██ ██ ██    ██ ██  ██ ██ ' \
                ' ██      ██ ██ ███████ ███████ ██  ██████  ██   ████ '
            ;;
        PARKING)
            printf '%s\n' \
                ' ██████   █████  ██████  ██   ██ ██ ███    ██  ██████  ' \
                ' ██   ██ ██   ██ ██   ██ ██  ██  ██ ████   ██ ██       ' \
                ' ██████  ███████ ██████  █████   ██ ██ ██  ██ ██   ███ ' \
                ' ██      ██   ██ ██   ██ ██  ██  ██ ██  ██ ██ ██    ██ ' \
                ' ██      ██   ██ ██   ██ ██   ██ ██ ██   ████  ██████  '
            ;;
    esac
    printf '\033[0m\n\033[1;32m                    ● RUNNING\033[0m\n'
}

# control과 포트 확인에는 로그 토글을 적용하지 않는다.
if [[ "$scenario" == control_p || "$scenario" == check_devices ]]; then
    echo "=== $title 실행 중 (종료: Ctrl+C) ==="
    "${command[@]}"
    status=$?
    if [[ "$scenario" == check_devices ]]; then
        echo
        read -r -p '결과를 확인한 뒤 Enter를 누르면 창이 닫힙니다...'
    fi
    exit "$status"
fi

mkdir -p "$log_dir"
logs_visible=true
[[ -f "$state_file" ]] && logs_visible=false
$logs_visible || show_banner

set -o pipefail
"${command[@]}" 2>&1 | while IFS= read -r line; do
    printf '%s\n' "$line" >>"$log_file"
    current_visible=true
    [[ -f "$state_file" ]] && current_visible=false

    if [[ "$current_visible" != "$logs_visible" ]]; then
        if $current_visible; then
            printf '\033[2J\033[H=== %s 상세 로그 ON ===\n' "$title"
        else
            show_banner
        fi
        logs_visible=$current_visible
    fi
    $current_visible && printf '%s\n' "$line"
done
status=${PIPESTATUS[0]}
printf '\n종료되었습니다 (코드: %s).\n' "$status"
exit "$status"
