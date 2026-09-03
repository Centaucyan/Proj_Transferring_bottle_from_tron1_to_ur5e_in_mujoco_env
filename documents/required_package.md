# Required Packages & Environment Setup

* **프로젝트명:** Proj_Transferring_bottle_from_tron1_to_ur5e_in_mujoco_env
* **기본 환경:** Ubuntu 22.04 LTS, ROS 2 Humble, Conda 가상환경 (`transfer_bottle_by_tron1_py3_10`, Python 3.10.x)

---

## 1. 기본 환경 및 런타임

| 구분 | 소프트웨어 / 도구 | 권장 버전 | 주요 역할 및 용도 | 설치 방법 |
| :--- | :--- | :--- | :--- | :--- |
| **OS** | Ubuntu Desktop | **22.04 LTS (Jammy)** | 기본 운영체제 | - |
| **가상환경** | **Conda** | **Python 3.10.x** | 가상환경명: `transfer_bottle_by_tron1_py3_10` (ROS 2 Humble 호환성 유지) | `conda create -n transfer_bottle_by_tron1_py3_10 python=3.10 -y` |
| **빌드 도구** | colcon, rosdep | 최신 버전 | ROS 2 워크스페이스 패키지 빌드 및 의존성 관리 | `sudo apt install python3-colcon-common-extensions python3-rosdep` |

---

## 2. 시뮬레이터 및 물리 엔진

| 구분 | 소프트웨어 / 라이브러리 | 권장 버전 | 주요 역할 및 용도 | 설치 방법 |
| :--- | :--- | :--- | :--- | :--- |
| **Simulator** | **MuJoCo** | **3.x (3.2.x 이상)** | 물리 시뮬레이션 엔진 (Tron1 보행 동역학 및 UR5e/그리퍼 접촉 물리) | `pip install mujoco` |
| **Viewer** | mujoco-python-viewer | 최신 버전 | Python에서 MuJoCo 대화형 시각화 GUI 제공 | `pip install mujoco-python-viewer` |
| **연동 (선택)** | mujoco_ros2_control | Humble 호환 | MuJoCo와 ROS 2 제어 인터페이스(ros2_control) 연결 | Git 클론 후 빌드 또는 커스텀 브리지 작성 |

---

## 3. 로봇 매니퓰레이션 및 모션 플래닝 (UR5e & 그리퍼)

| 구분 | 소프트웨어 / 라이브러리 | 권장 버전 | 주요 역할 및 용도 | 설치 방법 |
| :--- | :--- | :--- | :--- | :--- |
| **Motion Planning** | **MoveIt 2** | **Humble 배포판** | UR5e 충돌 회피 경로 계획, 역기구학(IK) 풀이 | `sudo apt install ros-humble-moveit` |
| **Python API** | moveit_py | Humble 배포판 | Python 스크립트에서 MoveIt 2 모션 플래닝 직접 제어 | `sudo apt install ros-humble-moveit-py` |
| **로봇 모델** | ur_description | Humble 배포판 | UR5e 로봇팔 URDF/메시 파일 제공 | `sudo apt install ros-humble-ur-description` |
| **수학/기구학** | NumPy / SciPy | 최신 안정화 | 좌표 변환 행렬, 쿼터니언 연산, 궤적 보간 | `pip install numpy scipy` |
| **좌표 변환** | transforms3d | 0.4.x 이상 | 오일러각, 쿼터니언, 회전행렬 간 자유로운 변환 | `pip install transforms3d` |

---

## 4. 비전 및 3D 포인트 클라우드 처리 (Realsense D435i)

| 구분 | 소프트웨어 / 라이브러리 | 권장 버전 | 주요 역할 및 용도 | 설치 방법 |
| :--- | :--- | :--- | :--- | :--- |
| **2D 비전** | **OpenCV** | **4.x** | RGB 영상 처리, 색상/형태 기반 객체 검출 | `pip install opencv-python` |
| **ROS 변환** | cv_bridge | Humble 배포판 | ROS 2 `sensor_msgs/Image` ↔ OpenCV `ndarray` 변환 | `sudo apt install ros-humble-cv-bridge` |
| **3D 비전** | **Open3D** *(권장)* | **0.17.x ~ 0.18.x** | Depth 데이터 기반 포인트클라우드 평면 분할(RANSAC), 클러스터링, **병의 3D 기하학적 중심점(Centroid) 계산** | `pip install open3d` |
| **포인트 변환** | sensor_msgs_py | Humble 배포판 | ROS 2 `PointCloud2` 메시지 ↔ NumPy 변환 지원 | `sudo apt install ros-humble-sensor-msgs-py` |

> **참고 (PCL vs Open3D):**  
> Python 기반으로 비전 및 중심점 추출 파이프라인을 작성할 경우, 설치가 간편하고 NumPy와 호환성이 뛰어난 **Open3D**를 사용하는 것을 권장합니다.

---

## 5. 상태 머신 및 시나리오 오케스트레이션 (FSM)

| 구분 | 소프트웨어 / 라이브러리 | 권장 버전 | 주요 역할 및 용도 | 설치 방법 |
| :--- | :--- | :--- | :--- | :--- |
| **State Machine** | **py_trees** / **py_trees_ros** | Humble 배포판 | 비동기 파이프라이닝(3.6.1 & 3.6.2 병렬 수행) 및 예외 처리 FSM 제어 | `sudo apt install ros-humble-py-trees ros-humble-py-trees-ros` |
| **대체제** | smach / 커스텀 ROS 액션 | 최신 | 경량 상태 머신 또는 ROS 2 Action Server 기반 제어 | `pip install smach` |

---

## 6. Conda 가상환경 기반 설치 및 설정 절차 (`transfer_bottle_by_tron1_py3_10`)

### 6.1. 시스템 업데이트 및 ROS 2 필수 패키지 설치 (apt)
```bash
sudo apt update
sudo apt install -y \
  ros-humble-moveit \
  ros-humble-moveit-py \
  ros-humble-cv-bridge \
  ros-humble-sensor-msgs-py \
  ros-humble-ur-description \
  ros-humble-py-trees \
  ros-humble-py-trees-ros \
  python3-colcon-common-extensions \
  python3-rosdep
```

### 6.2. Conda 가상환경 생성 및 활성화
> **주의:** ROS 2 Humble과의 C++ 및 Python ABI 호환을 위해 반드시 **`python=3.10`**으로 생성해야 합니다.

```bash
# 1. 가상환경 생성
conda create -n transfer_bottle_by_tron1_py3_10 python=3.10 -y

# 2. 가상환경 활성화
conda activate transfer_bottle_by_tron1_py3_10

# 3. (권장) ROS 2 실행 시 CXXABI 버전 충돌 방지를 위한 libstdc++ 최신화
conda install -c conda-forge libstdcxx-ng -y
```

### 6.3. ROS 2 환경 오버레이
가상환경 활성화 후 시스템 ROS 2 Humble 설정을 로드합니다.
```bash
source /opt/ros/humble/setup.bash
```

### 6.4. Conda 가상환경 내 Python 핵심 라이브러리 설치
```bash
pip install --upgrade pip
pip install mujoco mujoco-python-viewer opencv-python open3d numpy scipy transforms3d pyyaml
```

### 6.5. 정상 연동 검증
가상환경 활성화 및 ROS 2 소싱 상태에서 아래 명령어로 라이브러리 임포트 정상 여부를 확인합니다:
```bash
python -c "import rclpy; import mujoco; import open3d; import cv2; print('✅ transfer_bottle_by_tron1_py3_10 환경 구성 완료!')"
```

