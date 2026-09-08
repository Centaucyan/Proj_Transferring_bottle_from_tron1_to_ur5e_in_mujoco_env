# Required Packages & Environment Setup

* **프로젝트명:** Proj_Transferring_bottle_from_tron1_to_ur5e_in_mujoco_env
* **문서 버전:** v2.0
* **기본 환경:** Ubuntu 22.04 LTS, ROS 2 Humble, Conda 가상환경 (`transfer_bottle_by_tron1_py3_10`, Python 3.10.x)

---

## 1. 기본 환경 및 런타임

| 구분 | 소프트웨어 / 도구 | 권장 버전 | 주요 역할 및 용도 | 설치 방법 |
| :--- | :--- | :--- | :--- | :--- |
| **OS** | Ubuntu Desktop | **22.04 LTS (Jammy)** | 기본 운영체제 | - |
| **가상환경** | **Conda** | **Python 3.10.x** | 가상환경명: `transfer_bottle_by_tron1_py3_10` (ROS 2 Humble C++ ABI 및 Python 호환 유지) | `conda create -n transfer_bottle_by_tron1_py3_10 python=3.10 -y` |
| **빌드 도구** | colcon, rosdep | 최신 버전 | ROS 2 워크스페이스 패키지 빌드 및 시스템 의존성 관리 | `sudo apt install python3-colcon-common-extensions python3-rosdep` |
| **그래픽스 / 렌더링** | OpenGL, Mesa, GLFW | 최신 시스템 버전 | MuJoCo Python 바인딩(`mujoco.Renderer`) 헤드리스 및 오프스크린 RGB-D 렌더링 지원 | `sudo apt install libgl1-mesa-dev libgl1-mesa-glx libosmesa6-dev libglew-dev libglfw3 libglfw3-dev` |
| **설정 관리** | **PyYAML** | 최신 안정화 | 로봇 물리 파라미터, 관절 게인, 씬 설정 등의 YAML 설정 파일 로드 및 파싱 | `pip install pyyaml` |

---

## 2. 시뮬레이터 및 물리 엔진

| 구분 | 소프트웨어 / 라이브러리 | 권장 버전 | 주요 역할 및 용도 | 설치 방법 |
| :--- | :--- | :--- | :--- | :--- |
| **Simulator** | **MuJoCo** | **3.x (3.2.x 이상)** | 고속 물리 시뮬레이션 엔진 (Tron1 보행 동역학 및 UR5e/그리퍼 접촉 역학) | `pip install mujoco` |
| **Viewer** | mujoco-python-viewer | 최신 버전 | Python에서 MuJoCo 대화형 3D 시각화 GUI 제공 | `pip install mujoco-python-viewer` |
| **통신 아키텍처** | **Python Bridge** *(권장)* | 커스텀 노드 | `rclpy`와 `mujoco`를 직접 결합한 비동기 통신 브리지 (`src/sim_bridge/mujoco_ros_bridge.py`). C++ CMAKE 빌드 충돌 없이 안정적 구동 | Phase 03에서 직접 구현 |

> **참고 (C++ mujoco_ros2_control 배제 사유):**  
> `mujoco_ros2_control`은 C++ 기반의 빌드 플러그인으로 Conda 가상환경과 결합 시 심각한 ABI 충돌(컴파일 에러)을 유발하기 쉽습니다. 본 프로젝트는 학습 및 안정성을 위해 **순수 Python 브리지(`rclpy + mujoco`)**를 메인 아키텍처로 사용합니다.

---

## 3. 로봇 매니퓰레이션 및 모션 플래닝 (UR5e & 그리퍼)

| 구분 | 소프트웨어 / 라이브러리 | 권장 버전 | 주요 역할 및 용도 | 설치 방법 |
| :--- | :--- | :--- | :--- | :--- |
| **Motion Planning** | **MoveIt 2** | **Humble 배포판** | UR5e 충돌 회피 경로 계획, 역기구학(IK) 풀이 및 Python 연동 통합 메타패키지 (move_group, planning interface, moveit_msgs 포함) | `sudo apt install ros-humble-moveit` |
| **로봇 모델** | ur_description | Humble 배포판 | UR5e 로봇팔 URDF/메시 파일 제공 | `sudo apt install ros-humble-ur-description` |
| **모델 파서** | **xacro** | **Humble 배포판** | Tron1 및 UR5e Xacro 매크로를 URDF로 변환 및 MoveIt 연동 | `sudo apt install ros-humble-xacro` |
| **제어 메시지** | **control_msgs**<br>**trajectory_msgs** | **Humble 배포판** | UR5e 관절 궤적 제어(`JointTrajectory`) 및 Robotiq 2F-85 그리퍼 액션 제어(`GripperCommand`) 인터페이스 | `sudo apt install ros-humble-control-msgs ros-humble-trajectory-msgs` |
| **시각화 도구** | **rviz2** | **Humble 배포판** | 로봇 조인트 상태, 플래닝 경로, 포인트클라우드 및 TF 프레임 시각화 | `sudo apt install ros-humble-rviz2` |
| **수학/기구학** | NumPy / SciPy | 최신 안정화 | 좌표 변환 행렬, 쿼터니언 연산, 궤적 보간 | `pip install numpy scipy` |
| **좌표 변환** | transforms3d | 0.4.x 이상 | 오일러각, 쿼터니언, 회전행렬 간 자유로운 변환 | `pip install transforms3d` |
| **타입 검증** | typeguard | 최신 버전 | ROS 2 파라미터 생성 라이브러리(`generate-parameter-library-py`) 의존성 충족 | `pip install typeguard` |

---

## 4. 비전 및 3D 포인트 클라우드 처리 (Realsense D435i)

| 구분 | 소프트웨어 / 라이브러리 | 권장 버전 | 주요 역할 및 용도 | 설치 방법 |
| :--- | :--- | :--- | :--- | :--- |
| **2D 비전** | **OpenCV** | **4.x** | RGB 영상 처리, 색상/형태 기반 객체 검출 | `pip install opencv-python` |
| **ROS 변환** | cv_bridge | Humble 배포판 | ROS 2 `sensor_msgs/Image` ↔ OpenCV `ndarray` 상호 변환 | `sudo apt install ros-humble-cv-bridge` |
| **3D 비전** | **Open3D** *(권장)* | **0.17.x ~ 0.18.x** | Depth 데이터 기반 포인트클라우드 평면 분할(RANSAC), 클러스터링, **병의 3D 기하학적 중심점(Centroid) 계산** | `pip install open3d` |
| **포인트 변환** | sensor_msgs_py | Humble 배포판 | ROS 2 `PointCloud2` 메시지 ↔ NumPy 변환 지원 | `sudo apt install ros-humble-sensor-msgs-py` |
| **좌표계 변환** | **tf2_ros**<br>**tf2_geometry_msgs** | **Humble 배포판** | D435i 카메라 좌표계의 물병 3D 중심점(`PointStamped`)을 UR5e 로봇팔 베이스 좌표계(`ur5e_base`)로 실시간 변환 | `sudo apt install ros-humble-tf2-ros ros-humble-tf2-geometry-msgs` |
| **TF 연산 도구** | tf_transformations | Humble 배포판 | ROS 2 TF2와 연계된 쿼터니언/행렬 연산 지원 | `sudo apt install ros-humble-tf-transformations` |

---

## 5. 상태 머신 및 시나리오 오케스트레이션 (FSM)

| 구분 | 소프트웨어 / 라이브러리 | 권장 버전 | 주요 역할 및 용도 | 설치 방법 |
| :--- | :--- | :--- | :--- | :--- |
| **State Machine** | **py_trees**<br>**py_trees_ros** | **Humble 배포판** | 시나리오 3.6.1 & 3.6.2 비동기 병렬 파이프라이닝(Place와 사전 스캔 병렬 처리) 및 6대 예외 복구 오케스트레이션 | `sudo apt install ros-humble-py-trees ros-humble-py-trees-ros` |
| **BT 시각화** | **pydot** | 최신 버전 | `py_trees` 내부 트리 구조 시각화 및 DOT 그래프 렌더링 의존성 (Conda 가상환경 필수) | `pip install pydot` |
| **데이터 분석 (선택)** | matplotlib | 최신 안정화 | Tron1 보행 시 ZMP 안정도, 관절 토크, 정지 수렴 시간 2D 그래프 분석 | `pip install matplotlib` |

> **참고 (smach 배제 사유):**  
> 과거 ROS 1용 FSM 도구인 `smach`는 ROS 2 Humble과의 비동기 액션 연동이 불안정하고, 프로젝트의 핵심인 병렬 파이프라이닝(Parallel Node) 구현에 한계가 있어 본 프로젝트에서는 **`py_trees` (Behavior Tree)**로 단일화합니다.

---

## 6. Conda 가상환경 기반 설치 및 설정 절차 (`transfer_bottle_by_tron1_py3_10`)

### 6.1. 시스템 업데이트 및 필수 패키지 일괄 설치 (apt)
터미널에서 아래 명령을 실행하여 시스템 빌드 도구, 그래픽스 렌더링 라이브러리, ROS 2 관련 필수 패키지를 설치합니다:

```bash
sudo apt update
sudo apt install -y \
  python3-colcon-common-extensions \
  python3-rosdep \
  libgl1-mesa-dev \
  libgl1-mesa-glx \
  libosmesa6-dev \
  libglew-dev \
  libglfw3 \
  libglfw3-dev \
  ros-humble-rviz2 \
  ros-humble-moveit \
  ros-humble-cv-bridge \
  ros-humble-sensor-msgs-py \
  ros-humble-ur-description \
  ros-humble-xacro \
  ros-humble-control-msgs \
  ros-humble-trajectory-msgs \
  ros-humble-tf2-ros \
  ros-humble-tf2-geometry-msgs \
  ros-humble-tf-transformations \
  ros-humble-py-trees \
  ros-humble-py-trees-ros
```

#### 💡 apt 설치 패키지 주요 역할 및 기능 요약

| 패키지명 | 분류 | 본 프로젝트에서의 핵심 역할 및 목적 |
| :--- | :--- | :--- |
| **`python3-colcon-common-extensions`** | 빌드 도구 | ROS 2 워크스페이스의 커스텀 메시지, 인터페이스 및 노드 패키지를 빌드(`colcon build`)하는 표준 도구 세트 |
| **`python3-rosdep`** | 의존성 관리 | ROS 2 패키지의 `package.xml`에 명시된 시스템 종속성 라이브러리를 자동 검사 및 설치하는 관리 도구 |
| **`libgl1-mesa-dev`** | 렌더링 코어 | Mesa OpenGL 런타임의 기본 C/C++ 개발 헤더 및 공유 라이브러리 인터페이스 |
| **`libgl1-mesa-glx`** | 렌더링 코어 | X11 윈도우 디스플레이 시스템과 OpenGL 그래픽스 렌더링 컨텍스트를 연결하는 GLX 라이브러리 |
| **`libosmesa6-dev`** | 오프스크린 렌더링 | X-Server 디스플레이가 없는 백그라운드 환경에서도 CPU 기반으로 3D 장면을 렌더링하는 OSMesa 라이브러리 |
| **`libglew-dev`** | OpenGL 확장 | OpenGL Extension Wrangler Library. 고성능 셰이더 및 최신 OpenGL 확장 함수 포인터를 동적으로 탐색/로딩 |
| **`libglfw3`** | 창/입력 관리 | MuJoCo 뷰어 실행 시 크로스 플랫폼 윈도우 생성 및 키보드/마우스 상호작용 이벤트를 처리하는 런타임 |
| **`libglfw3-dev`** | 창/입력 관리 | GLFW3 라이브러리의 C/C++ 개발 헤더 파일 및 빌드용 심볼릭 링크 파일 |
| **`ros-humble-rviz2`** | 3D 시각화 | 로봇 모델(URDF), 센서 포인트클라우드, TF 좌표계, MoveIt 계획 궤적을 3D 그래픽으로 모니터링하는 ROS 2 시각화 도구 |
| **`ros-humble-moveit`** | 모션 플래닝 | UR5e 6축 매니퓰레이터의 기구학(IK/FK), 충돌 감지(OMPL), 모션 플래닝 메타 패키지 (move_group, planning interface, moveit_msgs 포함) |
| **`ros-humble-cv-bridge`** | 비전 인터페이스 | ROS 2 이미지 메시지(`sensor_msgs/msg/Image`)와 OpenCV 이미지 포맷(`numpy.ndarray`) 간의 무손실 고속 상호 변환 |
| **`ros-humble-sensor-msgs-py`** | 센서 데이터 | PointCloud2, LaserScan, Image 등 복잡한 센서 메시지 바이너리를 Python에서 처리하기 위한 유틸리티 |
| **`ros-humble-ur-description`** | 로봇 에셋 | Universal Robots(UR5e 등)의 공식 로봇 모델(URDF/XACRO) 및 3D 시각/충돌 메쉬(`*.dae`, `*.stl`) 파일 세트 |
| **`ros-humble-xacro`** | 로봇 모델링 | XML 매크로 파서. 복잡한 로봇 URDF를 변수화/모듈화하고 MuJoCo 변환용 평탄화(Flattening) URDF 생성 지원 |
| **`ros-humble-control-msgs`** | 제어 인터페이스 | 관절 궤적 추종(`FollowJointTrajectory`), 그리퍼 구동 명령 등 로봇 액터 제어 표준 액션/서비스 정의 |
| **`ros-humble-trajectory-msgs`** | 제어 인터페이스 | 시간에 따른 다관절 위치/속도/가속도/가크 목표점(`JointTrajectoryPoint`)을 표현하는 표준 메시지 |
| **`ros-humble-tf2-ros`** | 좌표계 변환 | Tron1 베이스, 트레이, D435i 카메라, UR5e 베이스, 엔드이펙터 간의 3D 공간 좌표계 트리 브로드캐스트/리스너 |
| **`ros-humble-tf2-geometry-msgs`** | 좌표계 변환 | Point, Pose, Vector3 등 기하학적 데이터 구조를 TF2 좌표 변환 파이프라인에서 직접 변환할 수 있는 헬퍼 |
| **`ros-humble-tf-transformations`** | 좌표계 변환 | 쿼터니언(Quaternion), 회전행렬, 오일러각 간의 수학적 상호 변환을 지원하는 ROS 2 표준 파이썬 라이브러리 |
| **`ros-humble-py-trees`** | 상태 머신 (BT) | 병렬 파이프라이닝(물병 전달 및 사전 스캔 동시 수행)과 6대 예외 복구를 관장하는 Behavior Tree 상태 머신 코어 |
| **`ros-humble-py-trees-ros`** | 상태 머신 (BT) | Behavior Tree 노드에서 ROS 2 토픽, 서비스, 액션을 비동기(Non-blocking)로 직접 호출하고 연동하는 확장 어댑터 |

---

### 6.2. Conda 가상환경 생성 및 활성화
> **주의 (CXXABI 호환성):**  
> ROS 2 Humble 바이너리는 시스템 기본 GCC(Ubuntu 22.04 기본 GCC 11.4)로 빌드되어 있습니다.  
> 반드시 **`python=3.10`**으로 가상환경을 생성해야 C-확장 모듈 ABI가 호환됩니다.

```bash
# 1. 가상환경 생성
conda create -n transfer_bottle_by_tron1_py3_10 python=3.10 -y

# 2. 가상환경 활성화
conda activate transfer_bottle_by_tron1_py3_10
```

> **[중요] CXXABI 충돌 여부 진단 및 상황별 조치 방법:**  
> Conda 환경을 활성화하면 Conda 내부 라이브러리 경로가 시스템보다 우선 탐색되므로, Conda의 `libstdc++.so.6` 버전이 시스템(Ubuntu 22.04 기본 GCC 11.4)보다 낮을 경우 `import rclpy` 시 `version GLIBCXX_3.4.30 not found` 에러가 발생할 수 있습니다.
> 
> * **1단계 (진단):** ROS 2 소싱 후 Phase 00 환경 진단 스크립트를 실행합니다.
>   ```bash
>   source /opt/ros/humble/setup.bash
>   python scripts_devel_roadmap/phase00_check_env.py
>   ```
> * **2단계 (조치):** 진단 결과에 따라 다음과 같이 조치합니다.
>   * **정상 (`OK`):** 라이브러리가 호환되므로 추가 조치 없이 바로 **6.3절**로 진행합니다.
>   * **심볼 충돌 에러 발생 시 (`GLIBCXX not found`):** 아래 방법 중 하나를 선택하여 해결합니다.
>     * **[방법 1 - 권장] Conda-Forge 최신 libstdc++ 설치:**
>       ```bash
>       conda install -c conda-forge libstdcxx-ng -y
>       ```
>     * **[방법 2 - 확실한 대안] 시스템 libstdc++ 심볼릭 링크 연결:**  
>       *(방법 1 이후에도 해결되지 않거나 버전 꼬임 발생 시, 시스템 검증 라이브러리를 직접 링크)*
>       ```bash
>       ln -sf /usr/lib/x86_64-linux-gnu/libstdc++.so.6 $CONDA_PREFIX/lib/libstdc++.so.6
>       ```
> 
> *(보다 상세한 CXXABI 동적 바인딩 이론 및 원리는 [development_roadmap/rm_phase00_runtime_binding.md](development_roadmap/rm_phase00_runtime_binding.md) 문서를 참고하세요.)*

### 6.3. ROS 2 환경 오버레이
가상환경 활성화 후 시스템 ROS 2 Humble 설정을 로드합니다.
```bash
source /opt/ros/humble/setup.bash
```

### 6.4. Conda 가상환경 내 Python 핵심 라이브러리 설치
```bash
python -m pip install --upgrade pip
python -m pip install mujoco mujoco-python-viewer opencv-python open3d numpy scipy transforms3d pyyaml matplotlib typeguard pydot onnxruntime
```

#### 💡 Python 핵심 패키지(pip) 상세 역할 및 기능 요약

| 패키지명 | 분류 | 본 프로젝트에서의 핵심 역할 |
| :--- | :--- | :--- |
| **`mujoco`** | 물리 시뮬레이터 | DeepMind의 고속 다물체 물리 엔진 코어. Tron1의 지면 접촉 및 UR5e/2F-85 그리퍼 접촉 동역학 연산 |
| **`mujoco-python-viewer`** | GUI 시각화 | Python에서 MuJoCo 씬을 실시간 3D 창으로 렌더링하고 시점 이동 및 물리 디버깅을 제공하는 인터랙티브 뷰어 |
| **`opencv-python`** | 2D 비전 | D435i 카메라의 2D 컬러(RGB) 영상 처리, 이미지 필터링 및 포맷 변환 지원 |
| **`open3d`** | 3D 비전 | Depth 데이터 기반 포인트 클라우드 생성, RANSAC 트레이 평면 분할 및 **물병의 3D 기하학적 중심점(Centroid) 계산** |
| **`numpy`** | 수치 연산 | 다차원 배열, 벡터 및 행렬 연산의 기본 코어 라이브러리 |
| **`scipy`** | 과학 계산 | 로봇 궤적 보간(Spline Interpolation), 공간 변환 및 수치 최적화 연산 |
| **`transforms3d`** | 좌표 변환 | 오일러각, 쿼터니언, 회전행렬, 동차변환행렬(Homogeneous Matrix) 간의 상호 변환 계산 |
| **`pyyaml`** | 환경 설정 | 로봇 물리 파라미터, 관절 게인, 씬 설정 등의 YAML 설정 파일 로드 및 파싱 |
| **`matplotlib`** | 데이터 시각화 | Tron1 보행 시 ZMP 궤적, 관절 토크 곡선, 정지 자세 안정화 수렴 시간 등을 2D 그래프로 플롯 분석 |
| **`typeguard`** | 런타임 타입 검증 | ROS 2 파라미터 생성 라이브러리(`generate-parameter-library-py`) 의존성 충족 및 타입 안전성 보장 |
| **`pydot`** | BT 시각화 | `py_trees` 내부의 트리 구조 시각화 및 DOT 그래프 렌더링 지원 (Conda 격리 환경 필수 의존성) |
| **`onnxruntime`** | 기계학습 추론 | LimX Dynamics 공식 사전 훈련 강화학습 정책(policy.onnx, encoder.onnx)을 CPU에서 500Hz로 실시간 추론하여 Tron1 제자리 발구름 및 균형 제어 |

---

### 6.5. 정상 연동 검증
가상환경 활성화 및 ROS 2 소싱 상태에서 아래 명령어로 주요 패키지의 정상 임포트 여부를 1차 확인합니다:
```bash
python -c 'import rclpy; import mujoco; import open3d; import cv2; import tf2_ros; import py_trees; import typeguard; import pydot; import onnxruntime; print("✅ transfer_bottle_by_tron1_py3_10 핵심 환경 구성 완료!")'
```
*(보다 심층적인 CXXABI 심볼 진단 및 오프스크린 렌더링 검사는 Phase 00의 `scripts_devel_roadmap/phase00_check_env.py`를 통해 진행합니다.)*

