# 향후 자율주행 알고리즘 테스트 환경

이 디렉터리는 원본 `expo` 브랜치의 Gazebo Classic 기반 ROS2 환경을 **새 인지·판단·제어 로직을 교체해 실험하기 위한 기반**으로 정리한 안내입니다. 전시 화면과 달리 특정 행사 운영을 전제로 하지 않습니다.

<sub>※ ROS 패키지 빌드는 Ubuntu / Humble CI에서 확인했습니다. 실제 Gazebo 주행과 GPU 추론은 별도 검증이 필요합니다.</sub>

## 공통 워크스페이스

코드는 `../src/` 한 곳에서 관리합니다. `exhibition/`과 `testing/`은 개별 ROS2 워크스페이스가 아니며, 같은 구현을 두 관점에서 설명합니다.

| 패키지 | 용도 |
| --- | --- |
| `simulation_pkg` | Gazebo 차량·동역학 모델, 센서, 시뮬레이션 제어 연결 |
| `camera_pkg` / `lidar_pkg` | 이미지·LiDAR 인지 노드 및 교체 대상 |
| `decision_making_pkg` | 차선 주행·미션·주차 판단 알고리즘 |
| `control_pkg` | 제어 관련 노드 |
| `interfaces_pkg` | 인지·판단·제어를 연결하는 사용자 정의 메시지 |
| `launch_pkg` | 일반·미션·주차 시나리오와 RViz 설정 |
| `exhibition_gui_pkg` / `exhibition_sim_plugins` | 전시 전용 GUI·센서 출력 최적화 확장 |

## 실험 시나리오

| 시나리오 | Launch | 실험 예시 |
| --- | --- | --- |
| 차선 주행 | `general_sim.py` | 차선 검출/추종, 횡방향 제어, 장애물 회피 및 노이즈 실험 |
| 장애물·신호등 | `mission_sim.py` | 센서 인지 지연, 장애물·신호등 상황 판단 |
| 수직주차 | `parking_sim.py` | 전·후방 인지, 주차 상태전이, 후진 경로 및 정렬 |

### 개념적 인터페이스

~~~text
Gazebo 차량 + 센서
    ↓ /cam0/image_raw 등
인지 노드 (camera_pkg, lidar_pkg)
    ↓ /cam0/detections 등
판단 노드 (decision_making_pkg)
    ↓ /motion_command (기본값)
시뮬레이션 제어 브리지
    ↓
Gazebo 차량 구동 및 피드백
~~~

`general_sim.py`는 `autonomy_motion_topic` 파라미터로 제어 토픽을 바꿀 수 있고, `show_exhibition_gui`·`show_gazebo_gui`·`show_rviz` launch 인자를 제공하므로 전시 GUI를 끄고 개발자용 화면을 사용할 수 있습니다. 단, 실제 지원 여부와 토픽 연결은 실행 전 재확인이 필요합니다.

## 실행 명령 참고 (검증 전)

~~~bash
cd projects/autonomous_simulation

# Ubuntu 22.04 + ROS2 Humble / Gazebo Classic 가정
# 누락된 binary mesh/texture/model은 README의 asset 복구 안내 참조
colcon build --symlink-install
source install/setup.bash

ros2 launch launch_pkg general_sim.py show_exhibition_gui:=false show_gazebo_gui:=true show_rviz:=true
ros2 launch launch_pkg mission_sim.py
ros2 launch launch_pkg parking_sim.py
~~~

실험 시에는 신규 노드를 `src/`에 추가하고 **원본 인지·판단·제어 인터페이스를 유지하면서 특정 노드/launch만 교체**하는 방식을 권장합니다. 전시용 코드를 지우거나 별도로 복제할 필요는 없습니다.

## 검증할 지표 (예정)

주행 경로 오차·충돌/미션 성공 여부, 속도·조향 응답, 인지 및 제어 지연, 초기 조건·센서 노이즈 변화에 따른 결과를 비교합니다. 이는 향후 실험 계획으로, **현재 저장소에서 측정된 성능 지표가 아닙니다.**

## 남은 제약

- 원본 메시·텍스처·학습 가중치의 일부는 용량/저작권 검토로 GitHub에 올리지 않았습니다. 누락된 자산이 있으면 정상 실행을 보장할 수 없습니다.
- `general_sim.py`는 기본적으로 `general_mission_with_crosswalk.pth` 모델을 참조합니다. `.pt`와 `.pth`를 같은 모델로 취급해서는 안 됩니다.
- 외부 ROS2 패키지와 GPU/CPU 구성, 차량 물리 파라미터가 결과에 영향을 미칩니다.
- ROS2 패키지 빌드는 CI에서 확인했습니다. Gazebo 주행, 센서 데이터 및 실차 연동 시험은 수행하지 않았습니다.

[공통 프로젝트 안내](../README.md) · [전시 기록](../exhibition/README.md)
