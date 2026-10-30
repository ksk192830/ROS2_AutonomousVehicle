# 라이선스 및 저작권 고지

이 저장소의 각 대회·연구 프로젝트는 저작권 및 라이선스 조건이 서로 다를 수 있습니다. 전체 저장소에 단일 MIT 라이선스가 일괄 적용되는 것은 아닙니다.

## 원본 자율주행 플랫폼 및 소스코드

본 프로젝트는 [성균관대학교 자동화연구실](https://micro.skku.ac.kr/micro/index.do)의 ROS 2 기반 자율주행 플랫폼 및 소스코드를 바탕으로 확장한 작업입니다.

- [SKKUAutoLab/ros2_autonomous_vehicle_book](https://github.com/SKKUAutoLab/ros2_autonomous_vehicle_book) — ROS 2 교재 실습 코드 (원본 저장소 GPL-3.0)
- [SKKUAutoLab/ros2_autonomous_vehicle_simulation](https://github.com/SKKUAutoLab/ros2_autonomous_vehicle_simulation) — 시뮬레이션 코드 (원본 저장소 MIT)

출처 표기만으로 원본 라이선스 의무가 대체되지는 않습니다. 재사용한 파일의 저작권 고지, 라이선스 사본 및 파생물 배포 조건을 별도로 확인해야 합니다.


## 시뮬레이션 코드

자율주행 시뮬레이션의 일부 코드는 Jinsun-Lee의 기존 MIT 고지를 포함합니다. 그 저작권 고지와 MIT 라이선스 본문은 [licenses/E2E_Paper_original_MIT.txt](licenses/E2E_Paper_original_MIT.txt)에 보존합니다. 해당 코드를 이관할 때 관련 폴더에도 적절한 고지를 유지합니다.

## 외부 자료

3D 모델, 이미지, 학습용 가중치 등은 각 자료의 라이선스와 2차 배포 조건을 확인합니다. 사용자 또는 공동 개발자가 직접 작성하지 않은 부분의 재배포 권한도 검토합니다.

논문에 기재된 공저자와 인용 문헌은 원본 PDF에 그대로 보존합니다.

## 제4회 대회 시뮬레이션 관련 사항

`projects/4th_skku_contest/src/simulation_pkg`에는 Jinsun-Lee가 유지보수자로 표시된 소스가 포함되어 있습니다. 기존 MIT 저작권 고지 원문은 함께 보존하지만, 해당 ROS 패키지 메타데이터에는 `GNU-GPL` 표기도 있어 **상호 적용 범위를 확인해야 합니다**. MIT 고지 하나로 해당 프로젝트 전체의 라이선스를 확정하지 않습니다.

차량 모델의 텍스처, 메시 및 외부 3D 에셋은 출처와 재배포 권한이 검증될 때까지 바이너리를 이관하지 않습니다.

## Expo 기반 자율주행 시뮬레이션

`projects/autonomous_simulation/LICENSE`에는 원본 코드에 포함됐던 **Jinsun-Lee (2024) MIT** 저작권 고지 원문을 그대로 보존합니다. 다만 `simulation_pkg/package.xml`의 `GNU-GPL`, `exhibition_gui_pkg/setup.py`의 `Apache-2.0` 메타데이터와 타사 Gazebo 에셋의 CC BY·CC0 조건은 함께 검토해야 합니다.

전시 GUI 예제 이미지의 저작권/라이선스 출처는 `projects/autonomous_simulation/src/exhibition_gui_pkg/assets/ASSET_CREDITS.md`, 개별 가구·소파 모델의 출처는 `projects/autonomous_simulation/src/simulation_pkg/models/`의 각 `ATTRIBUTION.md` 및 `SOFA_SOURCES.md`에 기록돼 있습니다. 메시·텍스처 등 원본 바이너리의 공개 재배포는 권리를 확인할 때까지 유보합니다.
