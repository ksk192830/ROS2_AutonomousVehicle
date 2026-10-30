# 프로젝트 비교

대회별 코드와 시뮬레이션은 독립 ROS 2 워크스페이스로 유지합니다. 같은 이름의 패키지라도 프로젝트별 구성이 다릅니다.

| 프로젝트 | 초점 | 구현에서 살펴볼 부분 | 성과·자료 |
| --- | --- | --- | --- |
| [제3회 성균관대](../projects/3rd_skku_contest/README.md) | 모듈형 실차 자율주행 | YOLO·BEV 차선 인식, LiDAR 8단계 주차 FSM, Arduino 연동 | SW부문 금상 · 2025.07.18 |
| [경기도 대회](../projects/gyeonggido_contest/README.md) | 다중 센서 주차 인지·제어 | 전·후방 카메라, LiDAR 클러스터링, 주차 지도와 상태 전이 | 대상 · 2026.02.11 |
| [제4회 성균관대](../projects/4th_skku_contest/README.md) | 실차·시뮬레이션 실행 구성 | YOLO·TensorRT 인지 노드, 미션별 launch, Gazebo·RViz 구성 | SW부문 대상 · 2026.07.30 |
| [Gazebo 시뮬레이션](../projects/autonomous_simulation/README.md) | 전시 경험과 알고리즘 실험 기반 | 운전자·관찰자 GUI, 장애물 상호작용, 차량·센서 플러그인 | Expo 구성·구현 기록 |

날짜는 상장 기재일입니다. [수상 자료](awards/README.md)에서 원본을 확인할 수 있습니다.

## 연구 자료

[모듈형 ROS 2 프레임워크](../papers/ksae25/README.md), [수직주차](../papers/ask26/README.md), [KSAE 2026 발표 확정 논문 2편](../papers/ksae26/README.md)은 별도의 논문 자료로 정리합니다. 논문의 실험 결과를 모든 프로젝트의 실행 성능으로 일반화하지 않습니다.

[검증 범위](validation.md) · [시스템 구조](architecture.md) · [포트폴리오 홈](../README.md)
