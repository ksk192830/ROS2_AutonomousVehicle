# Model Compression and Multiobjective Evaluation of RF-DETR for Parking Perception on a Laptop GPU

**분류:** KSAE 2026 (발표 확정)

**연구 내용:** RF-DETR Segmentation Large에서 유도한 26개 후보의 정밀도 변환, 가지치기, 2:4 희소성, 입력 해상도 감소 등을 비교했습니다. RTX 4050 Laptop GPU에서 BBox/Mask AP와 mIoU, 추론 지연, TensorRT 엔진 크기를 측정하고 정확도 게이트 통과 후 3목적 Pareto 분석을 수행했습니다.

**주요 결과:** 비지배 후보는 **C01**(구조 개선+FP16)과 **R01**(해상도 감소+FP16)입니다. C01은 Mask AP 0.6029, 중앙값 지연 25.300 ms, 엔진 64.7 MiB이며, R01은 가장 빠른 통과 후보로 24.010 ms를 기록했습니다.

**범위:** 측정 대상은 전체 ROS 2 주차 성공률이 아닌 단일 모델의 인지 정확도·배포 비용입니다.

**원문 PDF:** [rf-detr-compression.pdf](rf-detr-compression.pdf)
