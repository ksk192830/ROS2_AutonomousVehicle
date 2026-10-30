# E2E 자율주행 전시 설치 및 운영

## 새 Ubuntu 데스크탑 설치

이 프로젝트는 **Ubuntu 22.04**와 ROS 2 Humble을 기준으로 한다.

```bash
git clone <E2E_Paper 저장소 주소>
cd E2E_Paper
bash install.sh --auto
```

`install.sh` 하나가 다음 작업을 순서대로 수행한다.

- ROS 2 Humble과 Gazebo 설치
- NVIDIA GPU 감지 또는 CPU 모드 선택
- Python/RF-DETR 의존성 설치
- Git LFS 모델 파일 확인
- rosdep 의존성 설치와 전체 프로젝트 빌드
- 바탕화면에 **E2E 자율주행 전시** 실행 아이콘 생성
- 카메라 모델, CUDA, ROS 패키지 최종 검증

설치 후 바탕화면의 아이콘을 더블클릭하면 전체 전시 인터페이스가 실행된다. 다시 클릭하면 기존 실행을 안전하게 정리하고 처음부터 재시작한다.

## 전시 화면 조작

- `D`: 차량을 시작 위치로 되돌리고 장애물 3개를 새로 배치해 데모 시작
- `C`: 현재 목표 차선의 반대 차선으로 변경
- `1`, `2`, `3`: 각 장애물 생성/제거
- `F11`: 전체화면 전환
- `ESC` 또는 `Q`: 인터페이스 종료

차량이 충돌 등으로 6초 이상 진행하지 못하면 자동으로 시작 위치에 복구된다.

## 설치 확인 및 로그

```bash
bash install.sh --verify-only
```

가장 최근 실행 로그는 다음 파일에서 확인할 수 있다.

```text
~/.local/state/e2e-exhibition/latest.log
```
