#!/usr/bin/env bash

set -u

runtime_dir="${XDG_RUNTIME_DIR:-/tmp}/skku_4th"
state_file="$runtime_dir/terminal-logs-off"
mkdir -p "$runtime_dir"

if [[ -f "$state_file" ]]; then
    rm -f "$state_file"
    state="ON"
    message="일반·미션·주차의 상세 로그를 터미널에 표시합니다."
else
    : >"$state_file"
    state="OFF"
    message="일반·미션·주차 터미널에는 프로그램 이름만 표시합니다."
fi

if command -v notify-send >/dev/null 2>&1; then
    notify-send "터미널 상세 로그: $state" "$message"
fi
