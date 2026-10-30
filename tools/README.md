# 실행·검증 도구

프로젝트별 설치와 실행 순서는 [실행 환경 안내](../docs/reproduction.md)를 참고하세요. 명령은 저장소 최상위에서 실행합니다.

| 도구 | 용도 |
| --- | --- |
| `preflight.py` | 실행에 필요한 모델·메시·텍스처 검사 |
| `build_workspace.sh` | 독립 ROS 2 워크스페이스 빌드 |
| `import_assets.py` | 접근 권한이 있는 원본에서 로컬 실행 자산 복구 |
| `check_workspaces.sh` | 워크스페이스 구성 점검 |
| `add_3rd_model.sh` | 제3회 모델 로컬 설치(공개 배포 제외) |
| `import_4th_runtime_models.sh` | 제4회 실차 모델 복구 |
| `import_gyeonggido_assets.sh` | 경기도 대회 자산 복구 |
| `tests/` | 자산 검사·복구 도구 회귀 테스트 |

검사 실행: `python3 -m unittest discover -s tools/tests -v`
