#!/usr/bin/env bash

set -u

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
launcher="$script_dir/launch-scenario.sh"

selection="$(python3 "$script_dir/scenario-key-selector.py")" || exit 0

case "$selection" in
    general|mission|parking|control_p)
        setsid "$launcher" "$selection" </dev/null >/dev/null 2>&1 &
        ;;
    toggle_log)
        setsid "$script_dir/toggle-terminal-logs.sh" </dev/null >/dev/null 2>&1 &
        ;;
esac

# 실제 프로그램은 별도 세션에서 실행되므로 Alt+Z를 곧바로 다시 쓸 수 있다.
exit 0
