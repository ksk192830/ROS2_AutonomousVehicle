# Gazebo 자율주행 시뮬레이션

**Expo 체험형 전시 · 자율주행 알고리즘 실험 기반**

## 프로젝트 개요

Gazebo의 차량·카메라·LiDAR와 ROS 2 인지·판단·제어를 연결한 시뮬레이션입니다. 방문자가 자율주행을 체험하는 Expo 전시 구성과, 주행 로직을 교체해 살펴볼 수 있는 테스트 구성을 하나의 워크스페이스로 관리합니다.

**기술:** ROS 2 Humble · Gazebo Classic 11 · Camera / LiDAR · Qt · C++ 센서 플러그인

## 주요 구현

| 과제 | 구현 |
| --- | --- |
| 차량·센서 환경 | SDF/URDF 차량, 카메라·LiDAR, 트랙·주차 월드와 제어 브리지 |
| 시나리오별 주행 | 일반주행·장애물/신호등 미션·수직주차 launch |
| 방문자 체험 | 운전자·관찰자 화면, 게임패드, 장애물 배치와 차량 초기화 |
| 전시 운영 | GUI 런처, 로그·화면 모드, 센서 출력 조절용 C++ 플러그인 |

## 성과와 자료

- [Expo 전시 기록](exhibition/README.md): 운전자·관찰자 화면, 체험 기능과 조작 구성
- [알고리즘 테스트 안내](testing/README.md): 주행 시나리오, 토픽 인터페이스와 교체 지점
- 9개 ROS 2 패키지와 Gazebo C++ 플러그인의 CI 빌드, 3개 시뮬레이션 launch 인자 로딩을 확인했습니다. [검증 기록](../../docs/validation.md)

## 코드 안내

| 살펴볼 내용 | 코드 |
| --- | --- |
| 차량·월드·제어 브리지 | [simulation_pkg](src/simulation_pkg/) |
| 인지 | [camera_pkg](src/camera_pkg/) · [lidar_pkg](src/lidar_pkg/) |
| 판단·제어·메시지 | [decision_making_pkg](src/decision_making_pkg/) · [control_pkg](src/control_pkg/) · [interfaces_pkg](src/interfaces_pkg/) |
| 주행·전시 실행 구성 | [launch_pkg](src/launch_pkg/) · [scripts](scripts/) |
| 전시 화면·센서 플러그인 | [exhibition_gui_pkg](src/exhibition_gui_pkg/) · [exhibition_sim_plugins](src/exhibition_sim_plugins/) |

`exhibition/`과 `testing/`은 같은 `src/`를 설명하는 문서이며 별도 ROS 워크스페이스가 아닙니다.

## 실행 및 검증 범위

Ubuntu 22.04 / ROS 2 Humble / Gazebo Classic 11을 기준으로 합니다. **CI 빌드와 실행 구성 로딩을 확인했으며, GPU 추론·Gazebo 주행·전시 전체 흐름은 재검증하지 않았습니다.**

<details>
<summary>자산 복구와 실행 명령 참고</summary>

공개 저장소에는 일부 추론 가중치와 Gazebo 메시·텍스처가 없습니다. 원본 접근 권한을 가진 사용자가 `tools/import_assets.py`로 로컬 복구할 수 있습니다. 기존 복구 도구를 사용할 경우 기본 전시장 월드에는 가구도 필요하므로 `expo` 모드를 사용합니다.

```bash
# 자산 복구 후, 저장소 루트에서
python3 tools/preflight.py autonomous_simulation general_sim
bash tools/build_workspace.sh autonomous_simulation
source projects/autonomous_simulation/install/setup.bash
ros2 launch launch_pkg general_sim.py show_exhibition_gui:=false show_gazebo_gui:=true show_rviz:=true
# 미션: mission_sim.py / 주차: parking_sim.py
```

일반주행은 `general_mission_with_crosswalk.pth`를 사용합니다. `.pt`와 `.pth`를 같은 모델로 대체하지 않습니다. 원본의 [전시 설치 안내](EXHIBITION_SETUP.md)와 [설치 스크립트](install.sh)도 보존합니다.

</details>

[실행 환경 안내](../../docs/reproduction.md)에서 자산 복구 범위와 설치 절차를 확인할 수 있습니다. 공개 저장소만 복제해 즉시 전체 전시를 재현할 수 있는 상태는 아닙니다.

## 관련 문서

[전시 기록](exhibition/README.md) · [테스트 안내](testing/README.md) · [검증 기록](../../docs/validation.md) · [라이선스·크레딧](../../docs/license-and-credits.md) · [포트폴리오 홈](../../README.md)
