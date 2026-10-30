# 다른 컴퓨터에서 실행 준비하기

대상은 `4th_skku_contest`와 `autonomous_simulation`입니다. 기준 환경은 **Ubuntu 22.04, 시스템 Python 3.10, ROS 2 Humble, Gazebo Classic 11**입니다. 두 프로젝트는 패키지 이름이 같으므로 각각 빌드하고, 새 터미널에서 하나의 `install/setup.bash`만 source합니다.

## 1. 시스템 준비

[ROS 2 Humble 공식 설치 안내](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)에 따라 ROS desktop, ros-dev-tools를 설치합니다. Python pip, colcon, rosdep도 필요합니다. 공개 저장소는 학습 모델 가중치를 배포하지 않습니다. 원본 저장소가 Git LFS를 사용한다면 해당 원본에서만 별도로 설치하세요.

```bash
sudo apt-get install python3-pip python3-colcon-common-extensions python3-rosdep
# rosdep을 처음 설치한 PC에서만
sudo rosdep init
rosdep update --rosdistro humble
# 공개 저장소에서 git lfs pull로 내려받을 학습 가중치는 없습니다.
```

## 2. 비공개 실행 자산 복구

공개 저장소에는 학습된 AI 모델 가중치(`.pt`, `.pth` 등) 및 일부 3D 자산이 없습니다. 학습 가중치는 직접 학습하거나 접근 권한이 있는 원본에서 로컬에만 복구하세요. 추적 및 재배포하지 마세요. 제3회 모델은 `bash tools/add_3rd_model.sh /path/to/final.pt`로 로컬 설치할 수 있습니다. 원본 접근 권한이 있는 사용자가 원본을 로컬로 복제한 후 아래 명령을 사용합니다. 원본 주소는 공개 문서에 기록하지 않습니다. 시뮬레이션 원본은 `expo` 브랜치, 제4회 원본은 해당 대회 코드가 들어 있는 리비전을 사용합니다.

```bash
python3 tools/import_assets.py /path/to/fourth-source 4th_skku_contest --ref SOURCE_COMMIT
python3 tools/import_assets.py /path/to/expo-source autonomous_simulation --ref SOURCE_COMMIT
```

`SOURCE_COMMIT`은 원본의 `git rev-parse HEAD` 결과로 대체합니다. 부분 복제 저장소도 지원하며 원격 접근에는 인증된 `gh`가 필요합니다. Git blob/LFS 해시를 검사하고 다른 내용의 기존 파일은 덮어쓰지 않습니다. 새 바이너리는 로컬 Git 제외 목록에 등록하며 자동 커밋·업로드하지 않습니다. 재배포 권한이 확인되기 전까지 이 경로를 사용합니다.

기존 시뮬레이션 `core` 복구는 기본 월드의 전시장 가구를 모두 포함하지 않습니다. 기본 월드를 그대로 재현하려면 위 도구 또는 기존 도구의 `expo` 모드를 사용하세요.

## 3. 시나리오별 파일 점검

```bash
python3 tools/preflight.py 4th_skku_contest general_sim
python3 tools/preflight.py 4th_skku_contest mission_sim
python3 tools/preflight.py 4th_skku_contest parking_sim
python3 tools/preflight.py autonomous_simulation general_sim
python3 tools/preflight.py autonomous_simulation mission_sim
python3 tools/preflight.py autonomous_simulation parking_sim
```

누락 파일, 다운로드되지 않은 LFS 포인터, 별도 확인할 외부 참조가 있으면 종료 코드 1을 반환합니다. 이 검사는 선택한 모델과 월드에서 추적 가능한 SDF/OBJ/MTL/material 참조를 검사합니다. 플러그인 로딩, DAE 내부 텍스처, 동적으로 생성되는 모델, 물리 동작이나 모델 정확도까지 보장하지 않습니다. Gazebo 기본 `sun`, `ground_plane`, 기본 재질은 설치된 Gazebo가 제공합니다.

## 4. 빌드와 AI 런타임

```bash
bash tools/build_workspace.sh 4th_skku_contest
# 또는 새 터미널에서
bash tools/build_workspace.sh autonomous_simulation
```

빌드는 ROS 메시지·패키지·C++ 플러그인을 확인합니다. Python 노드의 AI 모듈은 실행 전에 별도로 준비해야 합니다. 시뮬레이션의 `install.sh`에는 원본 전시 환경의 PyTorch/RF-DETR/Ultralytics 버전이 기록되어 있습니다. 전시 설치 전체가 필요한 Ubuntu PC에서는 자산 복구 후 `bash projects/autonomous_simulation/install.sh --cpu` 또는 `--gpu`를 사용할 수 있습니다. 이 스크립트는 시스템 패키지와 사용자 Python 패키지, 실행 아이콘을 설치합니다. GPU 버전의 실제 호환성은 대상 드라이버에서 확인해야 합니다.

제4회 시뮬레이션의 YOLO 노드는 `torch`, `ultralytics`, `numpy`, `opencv-python`을 필요로 합니다. 원본 전시 환경의 버전은 호환성 확인의 출발점이며, 제4회에서 검증한 버전 잠금 파일은 아직 없습니다. 실행한 PC의 `python3 -m pip freeze`, `nvidia-smi` 결과를 검증 기록에 보존하세요.

## 5. 실행

```bash
source /opt/ros/humble/setup.bash
source projects/4th_skku_contest/install/setup.bash
ros2 launch launch_pkg general_sim.py
# mission_sim.py 또는 parking_sim.py도 각각 별도로 실행
```

시뮬레이션 프로젝트는 새 터미널에서 다음과 같이 실행합니다.

```bash
source /opt/ros/humble/setup.bash
source projects/autonomous_simulation/install/setup.bash
ros2 launch launch_pkg general_sim.py show_exhibition_gui:=false show_gazebo_gui:=true show_rviz:=true
```

`AV_SAMPLE_DIR`은 선택적인 입력/수집 이미지의 기준 폴더(기본 `~/ros2_av_samples`), `AV_CAR_IMAGE_PATH`는 선택적인 LiDAR 차량 표시 이미지입니다. 원본 샘플 촬영 데이터는 포함되지 않습니다.

## 제4회 실차 TensorRT 경로

`general.py`와 `mission.py`는 `track.engine`, `parking.py`는 `parking_front.engine` 및 `parking_rear.pt`를 사용합니다. `.engine`은 카메라 패키지의 `camera_pkg/model/`에 배치한 후 다시 빌드합니다. 엔진 파일은 Git에서 제외합니다.

실차 전방 엔진은 RF-DETR용입니다. YOLO `.pt`를 변환하거나 이름만 변경해서 대체하지 않습니다. 현재 노드는 고정 NCHW, batch 1 입력과 `dets`, `labels`, `masks` 출력을 요구합니다. 원본 ONNX/체크포인트, 클래스 순서, 입력 해상도, export 옵션과 대상 GPU/TensorRT 버전을 확정한 뒤 대상 PC에서 엔진을 생성해야 합니다. 다른 연구 저장소와 자동 연결하거나 모델을 자동 대체하지 않습니다.

## 검증 기록

GitHub workflow는 정적 검사, 독립 ROS 빌드, 설치된 일반·미션·주차 launch의 인자 로딩을 확인합니다. 모델·비공개 에셋·GPU·실차 동작 검증은 포함하지 않습니다. 실행 검증에서는 프로젝트 리비전, 원본 자산 리비전, 환경 버전, 모델 해시, 실행 명령, 센서 토픽 수신, 인지 출력, 차량 응답, 미션 결과와 로그를 남겨야 합니다. 실제 주행 검증 전에는 실차 장치 포트와 보정값도 확정해야 합니다.
