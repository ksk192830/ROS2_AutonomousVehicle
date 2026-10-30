# Simulation Pre-training for Vision-Based End-to-End Lane Keeping

**부제:** A Scaled-Vehicle Case Study with ROS2/Gazebo

**분류:** KSAE 2026 (발표 확정)

**연구 내용:** ROS 2/Gazebo Classic 환경에서 PilotNet을 시뮬레이션 영상으로 사전학습한 뒤, 실차 데이터로 파인튜닝했을 때의 차선 추종 성능을 실차 데이터만 사용한 학습과 비교했습니다.

**데이터:** Gazebo 영상 14,132장, 실차 영상 5,916장. 동일 학습률 조건에서 50 epoch 학습 비교.

**주요 결과:** 10⁻⁴ 학습률에서 차선 이탈 없는 주행 성공률 **93.3%(28/30)** 대 **53.3%(16/30)**. 2×10⁻⁵ 학습률에서는 **90%(27/30)** 대 **40%(12/30)**. 한 코스·일부 고정 시드에 대한 결과로 일반화는 제한됩니다.

**원문 PDF:** [simulation-pretraining.pdf](simulation-pretraining.pdf)
