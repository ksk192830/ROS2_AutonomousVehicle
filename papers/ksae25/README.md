# Development of a Modular ROS2 Software Framework for Autonomous Vehicles

**분류:** KSAE 2025

**주요 내용:** 카메라·LiDAR·판단·제어·사용자 정의 메시지 패키지를 독립적으로 구성하고, ROS 2 토픽을 통해 실차 자율주행 시스템을 연동한 연구입니다. 기록주행, 장애물·신호등 미션, 수직주차를 필요한 노드만 선택적으로 실행하는 구조로 구성했습니다.

**검증 결과:** 공식 기록주행 1분 32.67초, 미션주행 감점 없음. 수직주차는 초기 조건에 민감하여 공식 대회에서는 실패했으며, 이는 이후 주차 알고리즘 개선의 배경이 되었습니다.

**원문 PDF:** [modular-ros2-framework.pdf](modular-ros2-framework.pdf)

**관련 코드:** [제3회 자율주행 프로젝트](../../projects/3rd_skku_contest/README.md)
