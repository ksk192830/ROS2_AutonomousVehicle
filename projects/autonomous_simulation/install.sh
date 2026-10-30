#!/usr/bin/env bash

# One-file installer for the E2E autonomous-driving exhibition workstation.
# Ubuntu 22.04 + ROS 2 Humble is the supported exhibition environment.
if [[ -z "${BASH_VERSION:-}" ]]; then
  exec bash "$0" "$@"
fi

set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$SCRIPT_DIR"
PYTHON_BIN="/usr/bin/python3"
MODE="auto"
SKIP_APT=0
VERIFY_ONLY=0
DESKTOP_ONLY=0
REBOOT_REQUIRED=0

for arg in "$@"; do
  case "$arg" in
    --auto) MODE="auto" ;;
    --gpu) MODE="gpu" ;;
    --cpu) MODE="cpu" ;;
    --skip-apt) SKIP_APT=1 ;;
    --verify-only) VERIFY_ONLY=1 ;;
    --desktop-only) DESKTOP_ONLY=1 ;;
    -h|--help)
      printf '%s\n' \
        'Usage: bash install.sh [--auto|--gpu|--cpu] [--skip-apt] [--verify-only|--desktop-only]' \
        '  --auto         NVIDIA GPU를 감지하고 GPU/CPU 모드를 자동 선택(기본값)' \
        '  --gpu          CUDA용 PyTorch 설치' \
        '  --cpu          CPU용 PyTorch 설치' \
        '  --skip-apt     운영체제/ROS 패키지 설치 생략' \
        '  --verify-only  설치 상태만 검사' \
        '  --desktop-only 바탕화면 실행 아이콘만 다시 생성'
      exit 0
      ;;
    *)
      printf '[install][ERROR] 알 수 없는 옵션: %s\n' "$arg" >&2
      exit 1
      ;;
  esac
done

log() { printf '[install] %s\n' "$*"; }
warn() { printf '[install][WARN] %s\n' "$*" >&2; }
fail() { printf '[install][ERROR] %s\n' "$*" >&2; exit 1; }

on_error() {
  local exit_code=$?
  warn "설치가 중단되었습니다(단계: ${CURRENT_STEP:-unknown}, 코드: $exit_code)."
  warn "문제를 해결한 뒤 같은 install.sh를 다시 실행하면 이어서 설치됩니다."
  exit "$exit_code"
}
trap on_error ERR

require_supported_os() {
  CURRENT_STEP="운영체제 확인"
  [[ "$(id -u)" -ne 0 ]] || fail "root로 직접 실행하지 말고 일반 사용자로 실행해 주세요. sudo는 필요한 단계에서만 요청합니다."
  [[ -r /etc/os-release ]] || fail "Ubuntu 버전을 확인할 수 없습니다."
  source /etc/os-release
  if [[ "${ID:-}" != "ubuntu" || "${VERSION_ID:-}" != "22.04" ]]; then
    fail "지원 환경은 Ubuntu 22.04입니다. 현재: ${PRETTY_NAME:-unknown}"
  fi
  [[ -x "$PYTHON_BIN" ]] || fail "$PYTHON_BIN 을 찾을 수 없습니다."
  local py_version
  py_version="$($PYTHON_BIN -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')"
  [[ "$py_version" == "3.10" ]] || fail "ROS Humble용 Python 3.10이 필요합니다. 현재: $py_version"
  log "환경 확인 완료: ${PRETTY_NAME}, Python $py_version"
}

install_ros_and_system_packages() {
  CURRENT_STEP="ROS 2와 시스템 패키지 설치"
  if [[ "$SKIP_APT" -eq 1 ]]; then
    log "운영체제 패키지 설치를 건너뜁니다(--skip-apt)."
    return
  fi

  sudo apt-get update
  sudo apt-get install -y \
    locales software-properties-common curl gnupg lsb-release ca-certificates

  sudo locale-gen en_US en_US.UTF-8
  sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
  sudo add-apt-repository universe -y
  sudo apt-get update
  sudo apt-get install -y \
    git git-lfs build-essential cmake pkg-config pciutils \
    python3-pip python3-dev python3-venv python3-rosdep \
    python3-colcon-common-extensions python3-pyqt5 \
    libgl1 libglib2.0-0 libsm6 libxext6 libxrender1 libgtk-3-0 \
    libgomp1 libxcb-xinerama0 libnotify-bin zenity xdg-utils fonts-noto-cjk

  if [[ ! -e /etc/apt/sources.list.d/ros2.sources ]]; then
    if [[ ! -f /usr/share/keyrings/ros-archive-keyring.gpg ]]; then
      curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg
    fi
    local architecture
    architecture="$(dpkg --print-architecture)"
    printf 'deb [arch=%s signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main\n' "$architecture" \
      | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null
  else
    log "기존 ros2-apt-source 저장소 설정을 사용합니다."
  fi

  sudo apt-get update
  sudo apt-get install -y \
    ros-humble-desktop \
    ros-humble-gazebo-ros-pkgs ros-humble-gazebo-msgs \
    ros-humble-gazebo-plugins ros-humble-gazebo-ros2-control \
    ros-humble-ros2-control ros-humble-ros2-controllers \
    ros-humble-controller-manager ros-humble-controller-manager-msgs \
    ros-humble-joint-state-broadcaster ros-humble-position-controllers \
    ros-humble-velocity-controllers ros-humble-xacro ros-humble-cv-bridge \
    ros-humble-joy

  if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
    sudo rosdep init
  fi
  rosdep update
}

detect_compute_mode() {
  CURRENT_STEP="GPU 확인"
  local has_nvidia_hardware=0
  if command -v lspci >/dev/null 2>&1 && lspci | grep -qi 'NVIDIA'; then
    has_nvidia_hardware=1
  fi

  if [[ "$MODE" == "auto" ]]; then
    if command -v nvidia-smi >/dev/null 2>&1 && nvidia-smi >/dev/null 2>&1; then
      MODE="gpu"
    elif [[ "$has_nvidia_hardware" -eq 1 ]]; then
      MODE="gpu"
      if [[ "$SKIP_APT" -eq 0 && "$VERIFY_ONLY" -eq 0 ]]; then
        log "NVIDIA GPU는 있지만 드라이버가 없어 권장 드라이버를 설치합니다."
        sudo apt-get install -y ubuntu-drivers-common
        sudo ubuntu-drivers install
        REBOOT_REQUIRED=1
      else
        warn "NVIDIA 드라이버가 아직 동작하지 않습니다. 설치 후 재부팅이 필요합니다."
      fi
    else
      MODE="cpu"
    fi
  elif [[ "$MODE" == "gpu" && "$has_nvidia_hardware" -eq 0 ]]; then
    warn "NVIDIA GPU가 감지되지 않았지만 --gpu 요청에 따라 CUDA용 패키지를 설치합니다."
  fi
  log "연산 모드: $MODE"
}

install_python_packages() {
  CURRENT_STEP="AI Python 패키지 설치"
  local pip_cmd=("$PYTHON_BIN" -m pip)
  "${pip_cmd[@]}" install --user --upgrade pip wheel
  "${pip_cmd[@]}" install --user "setuptools==58.2.0"

  if [[ "$MODE" == "cpu" ]]; then
    "${pip_cmd[@]}" install --user \
      --index-url https://download.pytorch.org/whl/cpu \
      "torch==2.10.0" "torchvision==0.25.0"
  else
    if ! "${pip_cmd[@]}" install --user \
      --index-url https://download.pytorch.org/whl/cu128 \
      "torch==2.10.0" "torchvision==0.25.0"; then
      warn "CUDA 전용 저장소 설치가 실패해 기본 PyTorch 저장소로 다시 시도합니다."
      "${pip_cmd[@]}" install --user \
        "torch==2.10.0" "torchvision==0.25.0"
    fi
  fi

  "${pip_cmd[@]}" install --user \
    "numpy==1.26.4" \
    "opencv-python==4.11.0.86" \
    "pyserial==3.5" \
    "pynput==1.8.1" \
    "rfdetr==1.8.1" \
    "ultralytics==8.4.19" \
    "transformers==5.3.0" \
    "huggingface_hub==1.5.0"
}

prepare_project_assets() {
  CURRENT_STEP="모델과 Gazebo 자료 준비"
  cd "$WORKSPACE_DIR"
  git lfs install --local
  git lfs pull

  local model_file="$WORKSPACE_DIR/src/camera_pkg/camera_pkg/model/general_mission_with_crosswalk.pth"
  [[ -f "$model_file" ]] || fail "Segmentation 모델이 없습니다: $model_file"
  local model_size
  model_size="$(stat -c '%s' "$model_file")"
  (( model_size > 1048576 )) || fail "모델 파일이 Git LFS 포인터 상태입니다. 네트워크와 Git LFS를 확인해 주세요."

  local gazebo_models="$HOME/.gazebo/models"
  mkdir -p "$gazebo_models"
  cp -a "$WORKSPACE_DIR/src/simulation_pkg/models/." "$gazebo_models/"
  log "AI 모델 및 Gazebo 모델 준비 완료"
}

install_ros_dependencies_and_build() {
  CURRENT_STEP="ROS 프로젝트 빌드"
  set +u
  source /opt/ros/humble/setup.bash
  set -u
  cd "$WORKSPACE_DIR"
  rosdep install --from-paths src --ignore-src -r -y
  colcon build --symlink-install --event-handlers console_direct+
}

write_runtime_environment() {
  CURRENT_STEP="실행 환경 저장"
  local config_dir="$HOME/.config/e2e-exhibition"
  local env_file="$config_dir/env.sh"
  mkdir -p "$config_dir"
  {
    printf '# Generated by %s/install.sh\n' "$WORKSPACE_DIR"
    printf 'export E2E_WORKSPACE=%q\n' "$WORKSPACE_DIR"
    printf 'export E2E_COMPACT_LOGS=1\n'
    printf 'export E2E_PERFORMANCE_MODE=1\n'
    if [[ "$MODE" == "gpu" ]]; then
      printf 'export E2E_SEG_DEVICE=cuda:0\n'
    else
      printf 'export E2E_SEG_DEVICE=cpu\n'
    fi
  } > "$env_file"

  local bashrc_line='[[ -f "$HOME/.config/e2e-exhibition/env.sh" ]] && source "$HOME/.config/e2e-exhibition/env.sh"'
  if ! grep -Fq '.config/e2e-exhibition/env.sh' "$HOME/.bashrc" 2>/dev/null; then
    printf '\n# E2E autonomous-driving exhibition\n%s\n' "$bashrc_line" >> "$HOME/.bashrc"
  fi
}

create_desktop_launcher() {
  CURRENT_STEP="바탕화면 실행 아이콘 생성"
  local launcher="$WORKSPACE_DIR/scripts/start_exhibition.sh"
  local race_launcher="$WORKSPACE_DIR/scripts/start_race_game.sh"
  local icon="$WORKSPACE_DIR/src/exhibition_gui_pkg/assets/steering_wheel.png"
  local applications_dir="$HOME/.local/share/applications"
  local desktop_dir
  desktop_dir="$(xdg-user-dir DESKTOP 2>/dev/null || true)"
  [[ -n "$desktop_dir" ]] || desktop_dir="$HOME/Desktop"
  mkdir -p "$applications_dir" "$desktop_dir"
  chmod +x "$launcher" "$race_launcher"

  local desktop_file="$applications_dir/e2e-autonomous-driving-exhibition.desktop"
  {
    printf '%s\n' '[Desktop Entry]'
    printf '%s\n' 'Type=Application'
    printf '%s\n' 'Name=E2E 자율주행 전시'
    printf '%s\n' 'Comment=AI 자율주행 전시 인터페이스 시작 또는 재시작'
    printf 'Exec="%s"\n' "$launcher"
    printf 'Icon=%s\n' "$icon"
    printf '%s\n' 'Terminal=false'
    printf '%s\n' 'Categories=Education;'
    printf '%s\n' 'StartupNotify=true'
  } > "$desktop_file"
  chmod +x "$desktop_file"
  cp -f "$desktop_file" "$desktop_dir/E2E 자율주행 전시.desktop"
  chmod +x "$desktop_dir/E2E 자율주행 전시.desktop"
  gio set "$desktop_dir/E2E 자율주행 전시.desktop" metadata::trusted true \
    >/dev/null 2>&1 || true

  local race_desktop_file="$applications_dir/e2e-ai-ghost-race.desktop"
  {
    printf '%s\n' '[Desktop Entry]'
    printf '%s\n' 'Type=Application'
    printf '%s\n' 'Name=AI 고스트 레이스'
    printf '%s\n' 'Comment=AI 자율주행 기록과 겨루는 참여형 주행 게임'
    printf 'Exec="%s"\n' "$race_launcher"
    printf 'Icon=%s\n' "$icon"
    printf '%s\n' 'Terminal=false'
    printf '%s\n' 'Categories=Game;Education;'
    printf '%s\n' 'StartupNotify=true'
  } > "$race_desktop_file"
  chmod +x "$race_desktop_file"
  cp -f "$race_desktop_file" "$desktop_dir/AI 고스트 레이스.desktop"
  chmod +x "$desktop_dir/AI 고스트 레이스.desktop"
  gio set "$desktop_dir/AI 고스트 레이스.desktop" metadata::trusted true \
    >/dev/null 2>&1 || true
  update-desktop-database "$applications_dir" >/dev/null 2>&1 || true
  log "바탕화면 아이콘 생성 완료: E2E 자율주행 전시, AI 고스트 레이스"
}

verify_installation() {
  CURRENT_STEP="최종 설치 검증"
  set +u
  source /opt/ros/humble/setup.bash
  [[ -f "$WORKSPACE_DIR/install/setup.bash" ]] || fail "빌드 결과가 없습니다."
  source "$WORKSPACE_DIR/install/setup.bash"
  set -u

  "$PYTHON_BIN" - <<'PY'
import importlib
import sys

modules = ('numpy', 'cv2', 'torch', 'torchvision', 'rfdetr', 'PyQt5')
for name in modules:
    module = importlib.import_module(name)
    print(f"[verify] {name}: {getattr(module, '__version__', 'OK')}")

import torch
print(f"[verify] python: {sys.executable}")
print(f"[verify] torch.cuda.is_available: {torch.cuda.is_available()}")
if torch.cuda.is_available():
    print(f"[verify] GPU: {torch.cuda.get_device_name(0)}")
PY

  ros2 pkg prefix camera_pkg >/dev/null
  ros2 pkg prefix exhibition_gui_pkg >/dev/null
  ros2 pkg prefix launch_pkg >/dev/null
  ros2 pkg executables exhibition_gui_pkg | grep -q exhibition_dashboard
  ros2 pkg executables exhibition_gui_pkg | grep -q exhibition_race_game
  ros2 pkg executables joy | grep -q joy_node
  [[ -x "$WORKSPACE_DIR/scripts/start_exhibition.sh" ]] || fail "전시 실행 스크립트가 없습니다."
  [[ -x "$WORKSPACE_DIR/scripts/start_race_game.sh" ]] || fail "게임 실행 스크립트가 없습니다."
  log "ROS 패키지, GUI, AI 런타임 검증 완료"

  if [[ "$MODE" == "gpu" && "$REBOOT_REQUIRED" -eq 0 ]]; then
    if ! "$PYTHON_BIN" -c 'import torch; raise SystemExit(0 if torch.cuda.is_available() else 1)'; then
      warn "GPU 모드이지만 CUDA가 아직 활성화되지 않았습니다. NVIDIA 드라이버를 확인해 주세요."
    fi
  fi
}

main() {
  require_supported_os
  if [[ "$DESKTOP_ONLY" -eq 1 ]]; then
    create_desktop_launcher
    trap - ERR
    exit 0
  fi
  if [[ "$VERIFY_ONLY" -eq 1 ]]; then
    detect_compute_mode
    verify_installation
    exit 0
  fi

  local preflight="$WORKSPACE_DIR/../../scripts/preflight.py"
  [[ -f "$preflight" ]] || fail "통합 저장소의 scripts/preflight.py가 필요합니다."
  "$PYTHON_BIN" "$preflight" autonomous_simulation general_sim || fail "먼저 docs/reproduction.md에 따라 실행 자산을 복구해 주세요."
  install_ros_and_system_packages
  detect_compute_mode
  install_python_packages
  prepare_project_assets
  install_ros_dependencies_and_build
  write_runtime_environment
  create_desktop_launcher
  verify_installation

  trap - ERR
  printf '\n'
  log "설치가 완료되었습니다. 바탕화면의 'E2E 자율주행 전시' 아이콘을 더블클릭하세요."
  if [[ "$REBOOT_REQUIRED" -eq 1 ]]; then
    warn "NVIDIA 드라이버 적용을 위해 한 번 재부팅한 뒤 아이콘을 실행해 주세요."
  fi
}

main "$@"
