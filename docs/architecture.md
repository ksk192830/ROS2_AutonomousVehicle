# ROS 2 시스템 구조

각 프로젝트는 별도 워크스페이스이며, 이름이 같은 ROS 2 패키지가 있을 수 있습니다. 저장소 전체를 하나의 `colcon build` 대상으로 취급하지 않습니다.

~~~mermaid
flowchart LR
  A[Camera / LiDAR] --> B[Perception]
  B --> C[Decision / FSM]
  C --> D[Control]
  D --> E[Vehicle / Gazebo]
  E -. feedback .-> A
~~~

이 도식은 공통 개념을 보여줍니다. 프로젝트마다 사용 센서·노드와 제어 구성이 다릅니다.

## 구성

~~~text
projects/
  3rd_skku_contest/src/
  4th_skku_contest/src/
  gyeonggido_contest/src/
  autonomous_simulation/src/
papers/                 # ROS 2 빌드 대상 아님
~~~

실차 프로젝트는 카메라 장치 번호, 시리얼 포트, GPU, 모델 파일 경로를 환경별로 설정합니다. 시뮬레이션과 실차 실행 검증은 별도로 기록합니다.
