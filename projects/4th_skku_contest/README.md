# 제4회 성균관대학교 AI 자율주행 경진대회

**SW부문 대상 · 2026.07.30** · [상장 PDF](../../docs/awards/skku-4th.pdf)

## 프로젝트 개요

ROS 2의 인지·판단·제어 구조에 전·후방 영상 처리, LiDAR 주차 인지와 시나리오별 실행 구성을 결합한 자율주행 프로젝트입니다. 실차 제어와 Gazebo 시뮬레이션 구성을 함께 보존합니다.

**기술:** ROS 2 Humble · YOLO Segmentation · TensorRT · BEV · 2D LiDAR · FSM · Arduino

## 주요 구현

| 과제 | 구현 |
| --- | --- |
| 전·후방 인지 | YOLO 노드와 별도 RF-DETR TensorRT 엔진 노드, 차선·주차선·횡단보도·신호등 처리 |
| 시나리오별 판단 | 일반주행·미션·주차 노드를 분리하고 ROS 토픽으로 인지 결과 결합 |
| 주차 동작 | 전·후방 영상과 LiDAR 클러스터링을 이용한 후진·정렬·탈출 상태 제어 |
| 실차·시뮬레이션 구성 | Arduino 구동, Gazebo 차량 모델·월드·컨트롤러, RViz 시각화 |

## 성과와 자료

- **제4회 성균관대학교 AI 자율주행 경진대회 SW부문 대상.** [상장 미리보기](../../docs/awards/README.md)
- 7개 ROS 2 패키지와 일반주행·미션·주차 실행 구성, 차량 펌웨어·보조 도구를 보존했습니다.
- 통합 저장소의 Ubuntu / ROS 2 Humble CI 빌드와 시뮬레이션 launch 인자 로딩을 확인했습니다. [검증 기록](../../docs/validation.md)

## 코드 안내

| 살펴볼 내용 | 코드 |
| --- | --- |
| 카메라 인지·TensorRT 추론 | [camera_pkg](src/camera_pkg/) · [engine_seg.py](src/camera_pkg/camera_pkg/engine_seg.py) |
| LiDAR·주행 판단 | [lidar_pkg](src/lidar_pkg/) · [decision_making_pkg](src/decision_making_pkg/) |
| 차량 제어·메시지 | [control_pkg](src/control_pkg/) · [interfaces_pkg](src/interfaces_pkg/) |
| 실행 시나리오·Gazebo | [launch_pkg](src/launch_pkg/) · [simulation_pkg](src/simulation_pkg/) |
| 장치 점검·펌웨어 | [convenient](convenient/) · [arduino](arduino/) |

## 실행 및 검증 범위

Ubuntu 22.04 / ROS 2 Humble 기준의 독립 워크스페이스입니다. **CI 빌드와 실행 구성 로딩을 확인했으며, GPU 추론·Gazebo 주행·실차 동작은 재검증하지 않았습니다.**

<details>
<summary>모델과 실행 명령 참고</summary>

YOLO 모델 `track.pt`, `parking_front.pt`, `parking_rear.pt`, `crosswalk.pt`, `traffic_light_sim.pt`는 공개 저장소에 포함되지 않으며 로컬에서 별도로 준비해야 합니다.

실차 일반·미션은 `track.engine`, 실차 주차는 `parking_front.engine`을 추가로 요구합니다. 엔진은 환경별로 준비하며 YOLO `.pt`로 대체하지 않습니다. Gazebo 메시·텍스처는 원본 접근 권한으로 로컬 복구할 수 있습니다.

```bash
# 저장소 루트에서
 # 실행 모델을 별도로 준비한 후
bash tools/build_workspace.sh 4th_skku_contest
source projects/4th_skku_contest/install/setup.bash
ros2 launch launch_pkg general_sim.py
# 시뮬레이션 미션: mission_sim.py / 주차: parking_sim.py
# 실차 일반: general.py / 미션: mission.py / 주차: parking.py
```

실차 구성에서는 카메라 번호와 LiDAR·Arduino 포트를 확인합니다. `control_p` 제어 노드는 필요 시 별도로 실행합니다. 보조 이미지 입력은 `AV_SAMPLE_IMAGE`로 지정할 수 있습니다.

</details>

전체 절차와 누락 자산 범위는 [실행 환경 안내](../../docs/reproduction.md)를 참조합니다.

## 관련 문서

[프로젝트 비교](../../docs/project-comparison.md) · [검증 기록](../../docs/validation.md) · [이관 기록](../../docs/migration-notes.md) · [라이선스·크레딧](../../docs/license-and-credits.md) · [포트폴리오 홈](../../README.md)
