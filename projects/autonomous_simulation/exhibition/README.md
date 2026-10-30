# Expo 자율주행 전시 기록

**ROS2 + Gazebo 기반 자율주행 기술 체험형 전시**

이 문서는 `expo` 개발 버전에 구현된 **방문자 중심 자율주행 전시 경험**을 남기기 위해 작성했습니다. 차량·센서 시뮬레이션 코드는 [공통 `src/`](../src/)에서 관리하며, 향후 자율주행 알고리즘 시험에 관한 내용은 [별도 테스트 가이드](../testing/README.md)로 구분합니다.

<sub>※ ROS 2 패키지 빌드와 시뮬레이션 launch 인자 로딩은 CI에서 확인했습니다. 전시 전체 흐름은 재실행하지 않았습니다. [검증 기록](../../../docs/validation.md)</sub>

## 실제 대회장과 가상환경

| 실제 대회장 — 제4회 성균관대 | Expo 가상환경 |
| --- | --- |
| <img src="../../../docs/images/skku-4th-venue.png" alt="제4회 성균관대 실제 대회장" width="450"> | 사진 추가 예정 |

[실제 대회장 사진 출처: 대회 영상 4:06:05](https://www.youtube.com/watch?v=2yOSjMr1Qcw&t=14765s)

## 무엇을 보여주었나

- **인지 → 판단 → 제어 흐름**: 카메라 영상, 세그멘테이션, BEV/차선 시각화를 통해 자율주행 로직을 단계별로 이해하도록 구성
- **실시간 주행 체험**: Gazebo 차량과 체험형 GUI를 결합하여 일반주행과 차선 변경을 관찰
- **장애물 배치 및 안전 반응**: 관람객이 장애물을 생성·제거하고 주행 경로/차량 반응을 확인하는 인터랙션
- **게임패드 및 운전자/관찰자 화면**: 운전자 경험과 관찰자 설명 화면을 분리해 전시 운영
- **전시 운영 편의성**: 전시장 실행 스크립트, 화면 모드, 로그 확인 및 차량 재시작·초기화 기능

## 당시 전시 조작 구성

| 입력 | 동작 |
| --- | --- |
| `D` | 차량을 초기 위치로 재배치하고 장애물 3개를 배치해 데모 시작 |
| `C` | 목표 차선을 다른 차선으로 변경하도록 요청 |
| `1` / `2` / `3` | 해당 장애물 생성·제거 |
| `F11` | 전체화면 전환 |
| `ESC` / `Q` | 전시 UI 종료 |

진행 불능 상태가 지속되면 시작 위치로 복구하는 로직도 원본 구현에 포함돼 있습니다.

## 구현 파일

| 구분 | 주요 경로 |
| --- | --- |
| 방문자 GUI | [`exhibition_gui_pkg`](../src/exhibition_gui_pkg/) |
| Gazebo 전시용 플러그인 | [`exhibition_sim_plugins`](../src/exhibition_sim_plugins/) |
| 일반주행·듀얼 체험 Launch | [`general_sim.py`](../src/launch_pkg/launch/general_sim.py), [`race_game.py`](../src/launch_pkg/launch/race_game.py) |
| 전시 런처 | [`start_exhibition.sh`](../scripts/start_exhibition.sh), [`start_race_game.sh`](../scripts/start_race_game.sh) |
| 과거 환경 안내 | [`EXHIBITION_SETUP.md`](../EXHIBITION_SETUP.md) |
| 이미지·예제 출처 | [`ASSET_CREDITS.md`](../src/exhibition_gui_pkg/assets/ASSET_CREDITS.md) |

**주의:** 전시 상호작용 코드는 보존했지만, 모델 메시·텍스처·추론 가중치가 모두 올라간 상태는 아닙니다. 현재 GitHub 버전에서 전시를 그대로 재현할 수 있다고 주장하지 않습니다.

[공통 프로젝트 안내](../README.md) · [테스트 환경 안내](../testing/README.md)
