# 경기도 대학생 자율주행 경진대회

**대상 · 2026.02.11** · [상장 PDF](../../docs/awards/gyeonggido.pdf)

## 프로젝트 개요

전·후방 카메라와 LiDAR를 이용해 주행 상황과 주차 공간을 인지하고, 상태 전이에 따라 차량을 제어한 프로젝트입니다. 일반주행·미션·주차를 독립된 실행 구성으로 관리합니다.

**기술:** ROS 2 Humble · YOLO Segmentation · BEV · 2D LiDAR · FSM · Arduino

## 주요 구현

| 과제 | 구현 |
| --- | --- |
| 카메라 기반 인지 | 차선·장애물·신호등 및 전·후방 주차선 처리 |
| 주차 공간 탐색 | LiDAR 스캔, DBSCAN 클러스터링, 빈 공간 인지·시각화 |
| 상태 기반 판단 | 일반·미션·주차 판단 노드와 주차 상태 전이 |
| 차량 구동·관찰 | Arduino 제어, RViz와 지도 기반 주차 상황 시각화 |

## 성과와 자료

- **2026 경기도 대학생 자율주행 경진대회 대상.** [상장 미리보기](../../docs/awards/README.md)
- 카메라·LiDAR·판단·제어·메시지의 5개 ROS 2 패키지와 펌웨어·보조 도구를 보존했습니다.
- [주차 지도 이미지](convenient/parking_map/map_image.png)와 소스코드를 제공합니다. 실행 모델 가중치 3개는 공개하지 않습니다.

## 코드 안내

| 살펴볼 내용 | 코드 |
| --- | --- |
| 전·후방 카메라 인지 | [camera_pkg](src/camera_pkg/) |
| LiDAR 탐색·시각화 | [lidar_pkg](src/lidar_pkg/) |
| 주행·주차 판단 | [decision_making_pkg](src/decision_making_pkg/) |
| 실행 구성·차량 제어 | [launch](src/control_pkg/launch/) · [control_pkg](src/control_pkg/) |
| 사용자 정의 메시지 | [interfaces_pkg](src/interfaces_pkg/) |
| 주차 지도·장치 도구·펌웨어 | [convenient](convenient/) · [arduino](arduino/) |

## 실행 및 검증 범위

Ubuntu 22.04 / ROS 2 Humble 기준의 독립 워크스페이스입니다. `track.pt`, `parking_front.pt`, `parking_rear.pt`는 공개하지 않으며 실행 전 로컬에서 별도로 준비합니다. 통합 저장소에서 이 프로젝트의 ROS 빌드·실제 주행은 재검증하지 않았습니다.

<details>
<summary>실행 명령과 선택 자산 참고</summary>

```bash
# 저장소 루트에서
# 공개 저장소에 실행 모델 가중치는 없습니다
cd projects/gyeonggido_contest
colcon build --symlink-install
source install/setup.bash
ros2 launch control_pkg final.py
# 미션: final_mission.py / 주차: final_parking.py
```

LiDAR 차량 표시 이미지 `car.png`, 예제 촬영 이미지, CAD/STL은 포함하지 않았습니다. 보조 도구의 입력은 `AV_SAMPLE_IMAGE`, `AV_REAR_IMAGE_DIR`, `AV_CAR_IMAGE_PATH`로 지정할 수 있습니다. 모델 누락 복구용 도구는 저장소 루트의 `tools/import_gyeonggido_assets.sh`입니다.

</details>

수상은 대회 당시의 성과이며, 현재 코드의 실행 보장을 의미하지 않습니다. [검증 범위](../../docs/validation.md)

## 관련 문서

[프로젝트 비교](../../docs/project-comparison.md) · [시스템 구조](../../docs/architecture.md) · [이관 기록](../../docs/migration-notes.md) · [포트폴리오 홈](../../README.md)
