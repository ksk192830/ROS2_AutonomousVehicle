#!/usr/bin/env bash

set -u

case "${1:-}" in
    general) title="일반주행"; geometry="690x490+0+27" ;;
    mission) title="미션주행"; geometry="690x490+0+27" ;;
    parking) title="주차"; geometry="690x490+0+27" ;;
    # Start at the bottom-left immediately; run-scenario-terminal.sh then
    # corrects the position against the desktop work area (excluding panels).
    control_p) title="control_p"; geometry="690x490+0-0" ;;
    check_devices) title="포트 확인"; geometry="820x560+40+80" ;;
    *)
        echo "사용법: $0 {general|mission|parking|control_p|check_devices}" >&2
        exit 2
        ;;
esac

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec terminator \
    --title="$title" \
    --geometry="$geometry" \
    -x "$script_dir/run-scenario-terminal.sh" "${1:-}"
