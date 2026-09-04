# Proj_Transferring_bottle_from_tron1_to_ur5e_in_mujoco_env
* Update: 2026.09.04.
---

## 1. Description
* **개요:** MuJoCo 환경에서 Tron1(이족 보행 로봇)이 병을 인계 구역으로 이동 후 정지하면 UR5e(로봇팔)가 병을 인식한 후  Pick-and-Place 시스템 구현

* **목적:** MuJoCo 시뮬레이터와 ROS2를 기반으로 Tron1(이족 보행 로봇)을 제어하고, Moveit2 라이브러리 사용법, 객체 인식 및 Pick-and-Place 기능의 구현 프로세스 이해

* **HardWare:** 
    * **Robot:** LimX Dynamics Tron1(2족 보행 로봇), Universal Robots UR5e(로봇팔), Robotiq 2F-85(그리퍼)
    * **Sensor:**  Realsense D435i(RGB-D 카메라)

![work_space](documents/scenario/work_space_01.png)
---

## 2. 환경
* **OS:** Ubuntu 22.04 LTS
* **Language / Env:** Python 3.10.x (Conda 가상환경: `transfer_bottle_by_tron1_py3_10`)
* **Framework:** ROS2 Humble
* **Simulator:** MuJoCo 3.x
* **Core Libraries:** MoveIt 2, OpenCV, Open3D, py_trees
* **Tools:** Colcon, RViz2, MuJoCo Viewer
---

## 3. Pre-to-do
프로젝트 실행 전 아래 절차에 따라 의존성 패키지 설치 및 Conda 가상환경을 구축합니다.  
(자세한 패키지 구성 및 설명은 [documents/required_package.md](documents/required_package.md)를 참고하세요.)

### 3.1. 시스템 업데이트 및 필수 패키지 일괄 설치 (apt)
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
  ros-humble-moveit-py \
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

### 3.2. Conda 가상환경 생성 및 활성화
> **주의:** ROS 2 Humble과의 C++ 및 Python ABI 호환을 위해 반드시 **`python=3.10`**으로 생성합니다.

```bash
# 가상환경 생성 및 활성화
conda create -n transfer_bottle_by_tron1_py3_10 python=3.10 -y
conda activate transfer_bottle_by_tron1_py3_10
```

> **[중요] CXXABI 충돌 여부 진단 및 상황별 조치 방법:**  
> Conda 환경을 활성화하면 Conda 내부 라이브러리가 우선 탐색되므로, Conda의 `libstdc++.so.6` 버전이 시스템(Ubuntu 22.04)보다 낮을 경우 `import rclpy` 시 `version GLIBCXX_3.4.30 not found` 에러가 발생할 수 있습니다.
> 
> * **1단계 (진단):** ROS 2 소싱 후 Phase 00 환경 진단 스크립트를 실행합니다.
>   ```bash
>   source /opt/ros/humble/setup.bash
>   python scripts/check_env.py
>   ```
> * **2단계 (조치):** 진단 결과에 따라 다음과 같이 조치합니다.
>   * **정상 (`OK`):** 아무 조치도 필요 없으므로 바로 **3.3절**로 진행합니다.
>   * **심볼 충돌 에러 발생 시 (`GLIBCXX not found`):** 아래 방법 중 하나로 라이브러리를 최신화합니다.
>     * **[방법 1 - 권장] Conda-Forge libstdc++ 최신화:**
>       ```bash
>       conda install -c conda-forge libstdcxx-ng -y
>       ```
>     * **[방법 2 - 확실한 대안] 시스템 libstdc++ 심볼릭 링크 연결:**  
>       *(방법 1 이후에도 버전 불일치 시, 시스템 라이브러리를 직접 가상환경에 링크)*
>       ```bash
>       ln -sf /usr/lib/x86_64-linux-gnu/libstdc++.so.6 $CONDA_PREFIX/lib/libstdc++.so.6
>       ```
> 
> *(보다 상세한 CXXABI 동적 바인딩 이론 및 원리는 [documents/development_roadmap/rm_phase00_runtime_binding.md](documents/development_roadmap/rm_phase00_runtime_binding.md) 문서를 참고하세요.)*

### 3.3. ROS 2 환경 오버레이 및 Python 핵심 패키지 설치
```bash
# ROS 2 환경 로드
source /opt/ros/humble/setup.bash

# 가상환경 내 시뮬레이터 및 비전 라이브러리 설치
pip install --upgrade pip
pip install mujoco mujoco-python-viewer opencv-python open3d numpy scipy transforms3d pyyaml matplotlib
```

### 3.4. 환경 정상 연동 검증
```bash
python -c "import rclpy; import mujoco; import open3d; import cv2; import tf2_ros; import py_trees; print('✅ transfer_bottle_by_tron1_py3_10 핵심 환경 구성 완료!')"
```
---

## 4. Reference
* https://github.com/google-deepmind/mujoco_menagerie.git
---



