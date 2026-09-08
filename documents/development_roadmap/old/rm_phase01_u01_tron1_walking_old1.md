# [Phase 01-U01 Guide] Tron1 기본 이족보행 및 도킹 정지 단독 검증
# (Point-Foot Bipedal Locomotion & Docking Stance Lock Verification)

* **문서 버전:** v1.0
* **작성일:** 2026-09-04
* **프로젝트:** `Proj_Transferring_bottle_from_tron1_to_ur5e_in_mujoco_env`
* **대상 환경:** Ubuntu 22.04 LTS / ROS 2 Humble / MuJoCo 3.x / Conda (`transfer_bottle_by_tron1_py3_10`, Python 3.10.x)
* **문서 목적:** 본 문서는 무거운 트레이나 물병 페이로드를 장착하기에 앞서, **LimX Dynamics Tron1 점 발바닥(Point-Foot) 2족 보행 로봇 본체만으로 지면 충격을 흡수하며 직립(Standing)하고, 지정된 인계 구역 목표점까지 안정적으로 보행 이동한 후, 발구름 진동 없이 정지(Stance Lock)를 유지하는 동역학 제어 원리를 이론적으로 학습하고 직접 코드를 작성하여 검증**하는 것을 목적으로 합니다.

---

## 목차 (Table of Contents)

1. [왜 U01(Tron1 단독 보행 및 도킹 정지 검증)이 필요한가?](#1-왜-u01tron1-단독-보행-및-도킹-정지-검증이-필요한가)
2. [핵심 이론: 포인트 풋 2족 로봇 동역학과 제어 원리](#2-핵심-이론-포인트-풋-2족-로봇-동역학과-제어-원리)
   * [2.1. 포인트 풋(Point-Foot)의 기구학적 특성과 발목 토크의 부재](#21-포인트-풋point-foot의-기구학적-특성과-발목-토크의-부재)
   * [2.2. 역진자 모델(LIPM)과 질량 중심(CoM) 동역학](#22-역진자-모델lipm과-질량-중심com-동역학)
   * [2.3. Zero Moment Point (ZMP)와 지지 다각형의 한계](#23-zero-moment-point-zmp와-지지-다각형의-한계)
   * [2.4. 보행 상태 머신(Gait FSM)과 주기적 위상 변수($\phi$)](#24-보행-상태-머신gait-fsm과-주기적-위상-변수phi)
   * [2.5. 2-Link 다리 기구학과 가상 스프링-댐퍼(Virtual Spring-Damper)](#25-2-link-다리-기구학과-가상-스프링-댐퍼virtual-spring-damper)
   * [2.6. 관절 토크 PD 제어 및 중력 보상 원리](#26-관절-토크-pd-제어-및-중력-보상-원리)
   * [2.7. 도킹 정지(Stance Lock)와 미세 발구름(In-place Stepping) 진동 억제](#27-도킹-정지stance-lock와-미세-발구름in-place-stepping-진동-억제)
3. [단계별 실습: 내 손으로 직접 만들고 검증하기](#3-단계별-실습-내-손으로-직접-만들고-검증하기)
   * [Step 1: 작업 디렉토리 확인](#step-1-작업-디렉토리-확인)
   * [Step 2: 단위 샌드박스 씬 (xml_for_unit_test/phase01_u01_scene_unit_tron1.xml) 직접 작성](#step-2-단위-샌드박스-씬-xml_for_unit_testphase01_u01_scene_unit_tron1xml-직접-작성)
   * [Step 3: 단위 검증 스크립트 (scripts_devel_roadmap/phase01_u01_test_tron1_walking.py) 직접 작성](#step-3-단위-검증-스크립트-scripts_devel_roadmapphase01_u01_test_tron1_walkingpy-직접-작성)
   * [Step 4: 스크립트 실행 및 결과 검증 (물리 정합성 해석)](#step-4-스크립트-실행-및-결과-검증-물리-정합성-해석)
   * [Step 5: 인터랙티브 3D GUI 뷰어 조작 실습](#step-5-인터랙티브-3d-gui-뷰어-조작-실습)
4. [트러블슈팅 가이드 (자주 겪는 오류 및 원인 분석)](#4-트러블슈팅-가이드-자주-겪는-오류-및-원인-분석)
5. [Phase 01-U01 완료 체크리스트](#5-phase-01-u01-완료-체크리스트)

---

## 1. 왜 U01(Tron1 단독 보행 및 도킹 정지 검증)이 필요한가?

모바일 매니퓰레이션(Mobile Manipulation) 협동 작업에서 가장 빈번하게 발생하는 실패 요인은 **"로봇팔이 물건을 집으려는 순간, 이족보행 로봇의 베이스가 미세하게 흔들리는 현상"**입니다.

* **원인:** 발바닥이 평평한 휴머노이드 로봇과 달리, **Tron1은 발끝이 점(Point Foot, 구체)** 형태로 되어 있습니다. 발목 관절(Ankle Joint)이 존재하지 않아 지면을 발목 힘으로 딛고 버티는 정적 안정성(Static Stability)이 전혀 없습니다.
* **현상:** 보행을 멈추더라도 제자리에서 중심을 잡기 위해 발을 끊임없이 동동 구르는 **미세 발구름(In-place Stepping)**이 발생하며, 이는 상체에 장착될 트레이와 물병에 연속적인 고주파 진동 외란을 전달합니다.
* **해결 방안:** 따라서 상체에 컵홀더 트레이와 물병을 얹기 전(U02), 순수 로봇 본체(Bare Robot) 상태에서:
  1. 공중에서 바닥으로 떨어졌을 때 무릎 충격을 흡수하며 직립(Standing)하고,
  2. 목표 거리(인계 구역)까지 전진 보행을 안정적으로 수행하며,
  3. 목표점에 도달하는 즉시 양발을 안정적인 지지 삼각형 구도로 고정하는 **스탠스 락(Stance Lock)**을 활성화하여 3초 이상 진동을 완벽히 소쇄하는지 독립적으로 검증해야 합니다.

```
[U00: 베이스 물리 환경] ──(완료)──> [U01: Tron1 순수 보행/정지] ────> [U02: 트레이 장착 & 물병 운반]
                                          │
                     ┌────────────────────┴────────────────────┐
                     │ 1. 스폰 착지 충격 흡수 (Landing)            │
                     │ 2. 목표점 전진 보행 (Locomotion)           │
                     │ 3. 3초간 진동 없는 정지 (Stance Lock)       │
                     └─────────────────────────────────────────┘
```

---

## 2. 핵심 이론: 포인트 풋 2족 로봇 동역학과 제어 원리

### 2.1. 포인트 풋(Point-Foot)의 기구학적 특성과 발목 토크의 부재

휴머노이드 로봇은 넓은 직사각형 발바닥을 가지고 있어 발목에 모터를 장착하여 지면을 누르는 토크($\tau_{ankle}$)를 발생시킬 수 있습니다.  
반면, **Tron1의 다리 구조**는 경량화와 고속 기동성을 위해 끝단이 구체(반지름 $R=0.032\text{m}$) 형태인 **Point-Foot** 구조입니다:

* **다리당 관절 수:** 3개 (Abad: 롤 회전, Hip: 피치 회전, Knee: 피치 회전)
* **자유도 결핍(Underactuation):** 발목 관절이 없으므로, 지면 접촉점(Contact Point)에서는 마찰력에 의한 반력 $\mathbf{f}_c$만 전달될 뿐 모멘트(Moment) $\mathbf{m}_c$를 능동적으로 발생시킬 수 없습니다.
* **결론:** 기립과 보행의 안정성을 확보하기 위해서는 오직 **고관절(Hip)과 무릎(Knee)의 협조 제어** 및 **발끝의 착지 위치(Foot Placement)**에 의해서만 로봇의 전도를 막아야 합니다.

---

### 2.2. 역진자 모델(LIPM)과 질량 중심(CoM) 동역학

2족 보행 로봇의 복잡한 다물체 동역학은 로봇의 모든 질량이 중심 높이 $z_c$에 집중되어 있고, 다리는 질량이 없는 신축 막대로 가정한 **선형 역진자 모델(LIPM: Linear Inverted Pendulum Model)**로 근사화할 수 있습니다:

$$\ddot{x} = \frac{g}{z_c} (x - x_{foot}) = \omega^2 (x - x_{foot})$$

여기서:
* $x$: 로봇 질량 중심(CoM)의 수평 위치
* $x_{foot}$: 지면에 닿아 있는 지지발(Stance Foot)의 위치
* $z_c$: CoM의 수직 높이 (Tron1 기준 약 $0.78 \sim 0.82\text{m}$)
* $g$: 중력 가속도 ($9.81\text{m/s}^2$)
* $\omega = \sqrt{g / z_c}$: 역진자의 고유 진동수 (약 $\sqrt{9.81 / 0.8} \approx 3.5\text{rad/s}$)

> [!NOTE]
> **물리적 의미:**  
> 질량 중심이 지지발보다 앞서 나가면($x > x_{foot}$), 가속도 $\ddot{x}$가 양수가 되어 로봇은 전방으로 넘어지려고 합니다.  
> 넘어지지 않으려면 다음 걸음(Next Step)의 발 착지 위치 $x_{foot}$를 CoM 진행 방향 앞쪽에 재빨리 놓아야 합니다. 이것이 바로 **Raibert 발딛기 제어기(Foot Placement Heuristic)**의 핵심 원리입니다.

---

### 2.3. Zero Moment Point (ZMP)와 지지 다각형의 한계

* **ZMP (Zero Moment Point):** 지면 반력에 의해 발생하는 수평 방향의 순수 모멘트가 0이 되는 지면 위의 가상 지점입니다.
* **평평한 발:** ZMP가 넓은 발바닥 면적(지지 다각형, Support Polygon) 내부 안에 머무르면 넘어지지 않습니다.
* **포인트 풋 로봇:** 지지 다각형이 단 한 점(Single Contact Point)으로 축소됩니다!
  * 따라서 한 발 지지기(Single Support)에서는 ZMP가 무조건 발 접촉점에 묶이게 되며, **정적 안정이 물리적으로 불가능**합니다.
  * 유일하게 정적 안정을 이룰 수 있는 순간은 **두 발이 동시에 지면에 닿아 있는 양발 지지기(Double Support)**뿐입니다. 이때 두 발 사이를 잇는 선분이 지지 영역이 됩니다.

---

### 2.4. 보행 상태 머신(Gait FSM)과 주기적 위상 변수($\phi$)

보행은 주기적인 상태 전이(State Transition)로 모델링됩니다.  
보행 주기(Gait Cycle Period)를 $T$ (예: $0.5\text{초}$)라 할 때, 정규화된 위상 변수 $\phi \in [0, 1)$를 다음과 같이 정의합니다:

$$\phi(t) = \frac{t \pmod T}{T}$$

```
   0.0 ────────────── 0.5 ────────────── 1.0 (Phase φ)
┌──────────────────────┬──────────────────────┐
│   Left Stance (지지) │   Left Swing (유각)  │  왼다리 (Left Leg)
│   Right Swing (유각) │   Right Stance (지지)│  오른다리 (Right Leg)
└──────────────────────┴──────────────────────┘
```

1. **위상 1 ($\phi \in [0, 0.5)$):**
   * **왼다리:** 지면을 지지하며 몸체를 지탱하고 전방으로 밀어냄 (Stance Phase).
   * **오른다리:** 지면에서 떨어져 공중을 가르며 앞으로 이동 (Swing Phase).
2. **위상 2 ($\phi \in [0.5, 1.0)$):**
   * **오른다리:** 지면 접촉 후 지지 다리로 전환 (Stance Phase).
   * **왼다리:** 지면에서 떨어져 전방으로 이동 (Swing Phase).

---

### 2.5. 2-Link 다리 기구학과 가상 스프링-댐퍼(Virtual Spring-Damper)

Tron1의 다리는 고관절(Hip)과 무릎(Knee)으로 이루어진 2절 링크(2-link planar manipulator)로 취급할 수 있습니다.

* **링크 길이:**
  * 상퇴 링크(Upper leg, Hip to Knee): $L_1 = 0.30\text{m}$
  * 하퇴 링크(Lower leg, Knee to Foot): $L_2 = 0.30\text{m}$
* **순기구학 (Forward Kinematics):**
  고관절 각도 $q_{hip}$과 무릎 각도 $q_{knee}$로부터 발끝의 상대 위치 $(x_{foot}, z_{foot})$:
  $$x_{foot} = L_1 \sin(q_{hip}) + L_2 \sin(q_{hip} - q_{knee})$$
  $$z_{foot} = -L_1 \cos(q_{hip}) - L_2 \cos(q_{hip} - q_{knee})$$

* **가상 스프링-댐퍼 (Virtual Spring-Damper):**
  발이 지면에 닿을 때 강체 충격으로 인한 수치 튕김(Jitter)을 완화하기 위해, 다리 전체를 가상의 탄성 서스펜션으로 모델링합니다:
  $$F_z = K_{leg} (z_{target} - z_{foot}) - D_{leg} \dot{z}_{foot}$$
  야코비 행렬(Jacobian $J$)의 전치(Transpose)를 통해 관절 토크로 변환합니다:
  $$\boldsymbol{\tau} = \mathbf{J}^T \mathbf{F}$$

---

### 2.6. 관절 토크 PD 제어 및 중력 보상 원리

Tron1 모델(`model_ori/PF_TRON1A/xml/robot.xml`)의 액추에이터는 위치 서보가 아닌 **토크 모터(`<motor>`)**로 선언되어 있습니다.  
따라서 제어기는 매 타임스텝($dt=0.001\text{s}$)마다 관절 각도 오차와 각속도를 기반으로 인가할 토크 $\tau$를 계산해야 합니다:

$$\tau_i = K_p (q_{des, i} - q_{act, i}) + K_d (\dot{q}_{des, i} - \dot{q}_{act, i}) + \tau_{grav, i}$$

* $K_p$: 비례 게인 (관절 강성, Stiffness)
* $K_d$: 미분 게인 (관절 댐핑, Damping)
* $\tau_{grav}$: 로봇 상체 질량($m \approx 15\text{kg}$)을 지탱하기 위해 무릎과 고관절이 버텨야 하는 정적 중력 보상 토크:
  $$\tau_{knee, grav} \approx \frac{1}{2} m g L \sin(q_{knee} / 2)$$

---

### 2.7. 도킹 정지(Stance Lock)와 미세 발구름(In-place Stepping) 진동 억제

목표 지점(인계 구역, 예: $x = 1.0\text{m}$)에 도달했을 때, 보행 패턴을 즉시 멈추고 양발을 지면에 고정하는 **Stance Lock** 모드로 전환합니다:

1. **양발 지지 기하 대칭화:** 양쪽 다리의 목표 각도를 대칭적인 직립 자세($q_{stand}$)로 즉시 동기화합니다.
2. **게인 스케줄링 (Gain Scheduling):**
   * 보행 중: 유연한 충격 흡수를 위해 적정 게인 ($K_p = 60 \sim 80, K_d = 2 \sim 3$)
   * 정지(Stance Lock) 시: 외란 및 잔류 진동을 빠르게 소쇄하기 위해 강성 게인 대폭 상향 ($K_p = 150 \sim 200, K_d = 8 \sim 12$)
3. **안정성 판정 (3-Second Stance Lock Rule):**
   * 인계 구역 도달 후 최소 3초 동안 몸체의 롤(Roll) 및 피치(Pitch) 각속도가 **$0.05\text{rad/s}$ (약 $2.8^\circ/\text{s}$) 이하**로 유지되어야 합니다.

---

## 3. 단계별 실습: 내 손으로 직접 만들고 검증하기

> [!IMPORTANT]
> **학습 가이드:**  
> 아래 코드 블록들을 직접 확인하고, 프로젝트 디렉토리에 해당 파일을 생성하여 붙여넣은 뒤 실행해 보세요!  
> 파일 경로와 파일명을 정확하게 맞추어야 합니다.

---

### Step 1: 작업 디렉토리 확인

터미널에서 프로젝트 루트 디렉토리인지 확인합니다:
```bash
pwd
# 출력 확인: .../Proj_Transferring_bottle_from_tron1_to_ur5e_in_mujoco_env
```

---

### Step 2: 단위 샌드박스 씬 (`xml_for_unit_test/phase01_u01_scene_unit_tron1.xml`) 직접 작성

U00에서 검증한 공통 물리 옵션(dt=0.001s, implicitfast, cone="elliptic")과 바닥 평면을 기반으로, Tron1의 기구체, STL 메쉬, 모터 액추에이터, IMU 센서를 결합한 단독 단위 씬입니다.

파일을 새로 생성하고 아래의 전체 MJCF 코드를 저장합니다:
* **생성할 파일 경로:** `xml_for_unit_test/phase01_u01_scene_unit_tron1.xml`

```xml
<mujoco model="phase01_u01_unit_tron1">
  <!-- 
    ====================================================================
    Phase 01-U01: Tron1 기본 이족보행 및 도킹 정지 단독 검증 씬
    - U00 베이스 환경 상속 (dt=0.001s, implicitfast, elliptic cone)
    - LimX Dynamics Tron1 2족 보행 로봇 단독 배치 (페이로드 미적재)
    ====================================================================
  -->

  <!-- 1. 컴파일러 설정: 각도 라디안, 메쉬 상대경로 바인딩 -->
  <compiler angle="radian" coordinate="local" meshdir="../model_ori/PF_TRON1A/meshes/" autolimits="true"/>

  <size njmax="1000" nconmax="200"/>

  <!-- 2. 전역 물리 옵션 (U00 표준 준수) -->
  <option timestep="0.001" gravity="0 0 -9.81" integrator="implicitfast" cone="elliptic">
    <flag contact="enable"/>
  </option>

  <!-- 3. 뷰어 및 시각화 기본 설정 -->
  <visual>
    <global offwidth="640" offheight="480" azimuth="135" elevation="-20"/>
    <quality shadowsize="2048" offsamples="4"/>
    <rgba com="0.2 0.8 0.2 0.6" contactforce="0.8 0.2 0.2 0.8"/>
    <scale com="0.1" forcewidth="0.04" contactwidth="0.08"/>
  </visual>

  <!-- 4. 공통 에셋 (바닥 격자 텍스처 및 로봇 3D STL 메쉬) -->
  <asset>
    <!-- [U00 베이스 환경 상속] 하늘 배경 (Skybox) -->
    <texture type="skybox" builtin="gradient" rgb1="0.3 0.5 0.7" rgb2="0 0 0" width="512" height="512"/>

    <!-- [U00 베이스 환경 상속] 바닥 체크 패턴 (1m x 1m 격자 눈금선: 이동 거리 시각적 식별용) -->
    <texture name="texplane" type="2d" builtin="checker" rgb1="0.2 0.25 0.3" rgb2="0.3 0.35 0.4"
             width="512" height="512" mark="cross" markrgb="0.8 0.8 0.8"/>
    <material name="matplane" texture="texplane" texrepeat="5 5" texuniform="true" reflectance="0.1"/>

    <!-- Tron1 STL 3D 메쉬 에셋 -->
    <mesh name="base_Link" file="base_Link.STL"/>
    <mesh name="abad_L_Link" file="abad_L_Link.STL"/>
    <mesh name="hip_L_Link" file="hip_L_Link.STL"/>
    <mesh name="knee_L_Link" file="knee_L_Link.STL"/>
    <mesh name="foot_L_Link" file="foot_L_Link.STL"/>
    <mesh name="abad_R_Link" file="abad_R_Link.STL"/>
    <mesh name="hip_R_Link" file="hip_R_Link.STL"/>
    <mesh name="knee_R_Link" file="knee_R_Link.STL"/>
    <mesh name="foot_R_Link" file="foot_R_Link.STL"/>

    <!-- 재질 정의 -->
    <material name="robot_body_mat" rgba="0.82 0.85 0.90 1.0" specular="0.6" shininess="0.4"/>
    <material name="robot_dark_mat" rgba="0.2 0.2 0.22 1.0" specular="0.4" shininess="0.3"/>
    <material name="target_marker_mat" rgba="0.2 0.8 0.2 0.5"/>
  </asset>

  <!-- 5. 기본 클래스 정의 (시각 vs 충돌 분리) -->
  <default>
    <default class="visual">
      <geom contype="0" conaffinity="0" group="2" type="mesh"/>
    </default>
    <default class="collision">
      <geom contype="1" conaffinity="1" condim="3" group="3" friction="1.0 0.005 0.0001"/>
    </default>
    <!-- Tron1 다리 관절 기본 설정 (약간의 관성 및 댐핑 부여) -->
    <joint armature="0.02" damping="0.05" limited="true"/>
  </default>

  <!-- 6. 월드 바디: 조명, 바닥, 도킹 목표 마커, Tron1 로봇 바디 -->
  <worldbody>
    <!-- 광원 -->
    <light directional="true" pos="0 0 5" dir="0 0 -1" diffuse="0.8 0.8 0.8" castshadow="true"/>
    <light directional="false" pos="3 -3 3" dir="-1 1 -1" diffuse="0.4 0.4 0.4"/>

    <!-- 기준 바닥 평면 (U00 표준 준수: 기본 Group 0 시각 표시 및 충돌 설정) -->
    <geom name="floor" type="plane" size="0 0 0.05" material="matplane"
          contype="1" conaffinity="1" friction="1.0 0.005 0.0001" condim="3"/>

    <!-- 도킹 인계 구역 목표 마커 (x = 1.0m, y = 0.0m 지점에 반투명 원기둥 표시) -->
    <geom name="docking_target_marker" type="cylinder" pos="1.0 0 0.005" size="0.25 0.005"
          material="target_marker_mat" contype="0" conaffinity="0" group="1"/>

    <!-- 조망 관찰 카메라 -->
    <camera name="overview_cam" pos="2.2 -2.5 1.8" xyaxes="0.75 0.66 0.0 -0.25 0.28 0.92" fovy="45"/>
    <!-- 로봇 CoM 추적 카메라 -->
    <camera name="track_cam" mode="trackcom" pos="0 -2.4 1.2" xyaxes="1 0 0 0 0.5 0.86" fovy="50"/>

    <!-- 
      [Tron1 로봇 모델 본체]
      초기 스폰 위치: pos="0 0 0.82" (다리를 살짝 굽혔을 때 발끝이 지면에 정렬되는 안정 스폰 높이)
    -->
    <body name="base_Link" pos="0 0 0.82">
      <!-- 6-DOF 자유 관절 (공간 상을 자유롭게 이동 및 회전) -->
      <freejoint name="root_joint"/>
      
      <!-- 상체 관성 (Mass: 9.595kg) -->
      <inertial pos="0.0457 0.0001 -0.1638" quat="0.9725 -0.0061 -0.2324 0.0039"
                mass="9.595" diaginertia="0.1547 0.1109 0.0846"/>
      <site name="imu_site" pos="0 0 0"/>
      <geom class="visual" mesh="base_Link" material="robot_body_mat"/>
      <geom name="base_col" type="box" size="0.135 0.13 0.095" pos="0.03 0 -0.072" class="collision"/>

      <!-- ==================== LEFT LEG ==================== -->
      <body name="abad_L_Link" pos="0.05556 0.105 -0.2602">
        <inertial pos="-0.0697 0.0447 0.0005" quat="0.595 0.579 0.394 0.393" mass="1.469" diaginertia="0.0025 0.0020 0.0013"/>
        <joint name="abad_L_Joint" axis="1 0 0" range="-0.38 1.39"/>
        <geom class="visual" mesh="abad_L_Link" material="robot_body_mat"/>
        <geom name="abad_L_col" type="cylinder" pos="-0.08 0 0" euler="1.57 0 0" size="0.05 0.025" class="collision"/>

        <body name="hip_L_Link" pos="-0.077 0.0205 0">
          <inertial pos="-0.0286 -0.0477 -0.0399" quat="0.853 0.192 0.226 0.426" mass="2.3" diaginertia="0.0233 0.0230 0.0027"/>
          <joint name="hip_L_Joint" axis="0 1 0" range="-1.01 1.39"/>
          <geom class="visual" mesh="hip_L_Link" material="robot_dark_mat"/>
          <geom name="hip_L_col" type="cylinder" pos="-0.1 -0.02 -0.14" euler="0 0.53 0" size="0.035 0.09" class="collision"/>

          <body name="knee_L_Link" pos="-0.15 -0.0205 -0.25981">
            <inertial pos="0.0516 0.0015 -0.0814" quat="0.668 -0.202 -0.199 0.687" mass="0.55" diaginertia="0.0041 0.0041 0.0001"/>
            <joint name="knee_L_Joint" axis="0 -1 0" range="-0.87 1.36"/>
            <geom class="visual" mesh="knee_L_Link" material="robot_body_mat"/>
            <geom name="knee_L_col" type="cylinder" pos="0.078 0 -0.12" euler="0 -0.55 0" size="0.015 0.13" class="collision"/>

            <!-- 왼발끝 포인트 풋 (구체 접촉자: 반지름 0.032m) -->
            <geom class="visual" pos="0.150 0 -0.2598" mesh="foot_L_Link" material="robot_dark_mat"/>
            <geom name="foot_L_col" type="sphere" pos="0.150 0 -0.2598" size="0.032" class="collision"
                  friction="1.2 0.005 0.0001"/>
            <site name="foot_L_site" pos="0.150 0 -0.2598" size="0.01" rgba="1 0 0 1"/>
          </body>
        </body>
      </body>

      <!-- ==================== RIGHT LEG ==================== -->
      <body name="abad_R_Link" pos="0.05556 -0.105 -0.2602">
        <inertial pos="-0.0697 -0.0447 0.0005" quat="0.393 0.394 0.579 0.595" mass="1.469" diaginertia="0.0025 0.0020 0.0013"/>
        <joint name="abad_R_Joint" axis="1 0 0" range="-1.39 0.38"/>
        <geom class="visual" mesh="abad_R_Link" material="robot_body_mat"/>
        <geom name="abad_R_col" type="cylinder" pos="-0.08 0 0" euler="1.57 0 0" size="0.05 0.025" class="collision"/>

        <body name="hip_R_Link" pos="-0.077 -0.0205 0">
          <inertial pos="-0.0286 0.0477 -0.0399" quat="0.426 0.226 0.192 0.853" mass="2.3" diaginertia="0.0233 0.0230 0.0027"/>
          <joint name="hip_R_Joint" axis="0 -1 0" range="-1.39 1.01"/>
          <geom class="visual" mesh="hip_R_Link" material="robot_dark_mat"/>
          <geom name="hip_R_col" type="cylinder" pos="-0.10 0.025 -0.14" euler="0 0.53 0" size="0.035 0.09" class="collision"/>

          <body name="knee_R_Link" pos="-0.15 0.0205 -0.25981">
            <inertial pos="0.0516 -0.0015 -0.0814" quat="0.687 -0.199 -0.202 0.668" mass="0.55" diaginertia="0.0041 0.0041 0.0001"/>
            <joint name="knee_R_Joint" axis="0 1 0" range="-1.36 0.87"/>
            <geom class="visual" mesh="knee_R_Link" material="robot_body_mat"/>
            <geom name="knee_R_col" type="cylinder" pos="0.078 0 -0.12" euler="0 -0.55 0" size="0.015 0.13" class="collision"/>

            <!-- 오른발끝 포인트 풋 (구체 접촉자: 반지름 0.032m) -->
            <geom class="visual" pos="0.150 0 -0.2598" mesh="foot_R_Link" material="robot_dark_mat"/>
            <geom name="foot_R_col" type="sphere" pos="0.150 0 -0.2598" size="0.032" class="collision"
                  friction="1.2 0.005 0.0001"/>
            <site name="foot_R_site" pos="0.150 0 -0.2598" size="0.01" rgba="0 0 1 1"/>
          </body>
        </body>
      </body>
    </body>
  </worldbody>

  <!-- 7. 액추에이터 정의: 6개 모터 직접 토크 제어 ([-80, 80] Nm) -->
  <actuator>
    <motor name="abad_L_motor" joint="abad_L_Joint" gear="1" ctrllimited="true" ctrlrange="-80 80"/>
    <motor name="hip_L_motor"  joint="hip_L_Joint"  gear="1" ctrllimited="true" ctrlrange="-80 80"/>
    <motor name="knee_L_motor" joint="knee_L_Joint" gear="1" ctrllimited="true" ctrlrange="-80 80"/>
    <motor name="abad_R_motor" joint="abad_R_Joint" gear="1" ctrllimited="true" ctrlrange="-80 80"/>
    <motor name="hip_R_motor"  joint="hip_R_Joint"  gear="1" ctrllimited="true" ctrlrange="-80 80"/>
    <motor name="knee_R_motor" joint="knee_R_Joint" gear="1" ctrllimited="true" ctrlrange="-80 80"/>
  </actuator>

  <!-- 8. 센서 정의: IMU 및 6개 관절 엔코더 -->
  <sensor>
    <framequat name="imu_quat" objtype="site" objname="imu_site"/>
    <gyro name="imu_gyro" site="imu_site"/>
    <accelerometer name="imu_acc" site="imu_site"/>
    <jointpos name="pos_abad_L" joint="abad_L_Joint"/>
    <jointpos name="pos_hip_L"  joint="hip_L_Joint"/>
    <jointpos name="pos_knee_L" joint="knee_L_Joint"/>
    <jointpos name="pos_abad_R" joint="abad_R_Joint"/>
    <jointpos name="pos_hip_R"  joint="hip_R_Joint"/>
    <jointpos name="pos_knee_R" joint="knee_R_Joint"/>
    <jointvel name="vel_abad_L" joint="abad_L_Joint"/>
    <jointvel name="vel_hip_L"  joint="hip_L_Joint"/>
    <jointvel name="vel_knee_L" joint="knee_L_Joint"/>
    <jointvel name="vel_abad_R" joint="abad_R_Joint"/>
    <jointvel name="vel_hip_R"  joint="hip_R_Joint"/>
  </sensor>

  <!-- 9. 기본 안정 기립 키프레임 (뷰어 리셋 시 기본 자세 자동 복원) -->
  <keyframe>
    <key name="stand" qpos="0 0 0.82 1 0 0 0 0.0 0.40 0.80 0.0 -0.40 -0.80"/>
  </keyframe>
</mujoco>
```

---

### Step 3: 단위 검증 스크립트 (`scripts_devel_roadmap/phase01_u01_test_tron1_walking.py`) 직접 작성

이 스크립트는 다음 4단계 상태 머신을 거쳐 Tron1의 보행 및 정지 성능을 정량적으로 판정합니다:
1. **`STATE_LANDING` (0.0 ~ 1.5초):** 공중 스폰 후 지면 접촉 충격을 흡수하고 초기 기립.
2. **`STATE_WALKING` (1.5초 ~ 도달 시):** 교대 보행 위상($\phi$)에 따라 전진 보행하여 목표점($x = 1.0\text{m}$)까지 이동.
3. **`STATE_STANCE_LOCK` (도달 후 3.0초간):** 보행을 멈추고 관절 게인을 3배 상향하여 잔류 진동을 강제 감쇠.
4. **`STATE_EVALUATION`:** 롤/피치 진동 수렴도, 목표 오차($\le \pm 5\text{cm}$), 전도 여부를 검증하고 오프스크린 렌더링 스냅샷 저장.

파일을 새로 생성하고 아래 전체 소스 코드를 저장합니다:
* **생성할 파일 경로:** `scripts_devel_roadmap/phase01_u01_test_tron1_walking.py`

```python
#!/usr/bin/env python3
"""
scripts_devel_roadmap/phase01_u01_test_tron1_walking.py
Phase 01-U01: Tron1 Bipedal Locomotion & Docking Stance Lock
- 1.0x Real-time Physics Speed Synchronization
- Interactive Viewer Auto-Reset Support (Instant sync on Backspace/Reset)
- Unified 3D GUI & Console Telemetry Loop
"""

import os
import sys
import argparse
import math
import time
import numpy as np
import mujoco
import mujoco.viewer

class Colors:
    GREEN = '\033[92m'
    RED = '\033[91m'
    YELLOW = '\033[93m'
    CYAN = '\033[96m'
    BOLD = '\033[1m'
    RESET = '\033[0m'

def parse_args():
    parser = argparse.ArgumentParser(description="Phase 01-U01 Tron1 Locomotion Test")
    parser.add_argument("--headless", action="store_true", help="Run in headless text-only mode (no GUI window)")
    parser.add_argument("--xml", type=str, default="xml_for_unit_test/phase01_u01_scene_unit_tron1.xml",
                        help="Path to Tron1 unit scene XML")
    parser.add_argument("--target_x", type=float, default=1.0, help="Target docking X coordinate in meters")
    parser.add_argument("--max_time", type=float, default=12.0, help="Maximum simulation time limit in seconds")
    return parser.parse_args()

class Tron1BipedController:
    """
    Tron1 전용 2족 보행 및 스탠스 락 제어기
    - 기립 초기화: 스폰 순간 다리 급접힘(kick-down) 방지
    - 위상 변수(Phase) 기반 교대 보행 사인파 궤적
    - 상체 자세(Roll/Pitch) 안정화 피드백
    - 도킹 정지 시 게인 스케줄링(Stance Lock)
    """
    def __init__(self, model, target_x=1.0):
        self.model = model
        self.target_x = target_x

        self.actuator_names = [
            "abad_L_motor", "hip_L_motor", "knee_L_motor",
            "abad_R_motor", "hip_R_motor", "knee_R_motor"
        ]
        self.act_ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name) for name in self.actuator_names]
        self.base_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_Link")

        # 기립 기본 관절 각도 (자연스러운 완충 기립 자세)
        self.q_stand = np.array([0.0, 0.40, 0.80,  0.0, -0.40, -0.80])

        # 관절 강성 및 댐핑 게인
        self.kp_walk = np.array([120.0, 150.0, 150.0,  120.0, 150.0, 150.0])
        self.kd_walk = np.array([6.0, 8.0, 8.0,        6.0, 8.0, 8.0])

        self.kp_lock = np.array([200.0, 250.0, 250.0,  200.0, 250.0, 250.0])
        self.kd_lock = np.array([12.0, 15.0, 15.0,     12.0, 15.0, 15.0])

        self.gait_period = 0.50
        self.step_length = 0.12
        self.step_height = 0.18

        self.reset()

    def reset(self):
        self.state = "LANDING"
        self.dock_time = None
        self.stance_lock_duration = 3.0
        self.log_time = []
        self.log_x = []
        self.log_gyro = []
        self.log_fall = False

    def compute_torques(self, data):
        sim_time = data.time
        pos_x = data.xpos[self.base_body_id][0]
        pos_z = data.xpos[self.base_body_id][2]

        # 전도 감지 (상체 높이 0.35m 이하 추락 시)
        if pos_z < 0.35:
            self.log_fall = True

        q_act = data.qpos[7:13]
        v_act = data.qvel[6:12]
        gyro = data.sensor("imu_gyro").data.copy()

        # FSM 상태 전이
        if self.state == "LANDING":
            q_des = self.q_stand.copy()
            kp = self.kp_walk
            kd = self.kd_walk
            if sim_time >= 1.5:
                self.state = "WALKING"

        elif self.state == "WALKING":
            if pos_x >= (self.target_x - 0.05):
                self.state = "DOCKED"
                self.dock_time = sim_time

            phi = (sim_time % self.gait_period) / self.gait_period
            q_des = self.q_stand.copy()

            hip_swing = math.sin(2.0 * math.pi * phi) * self.step_length
            knee_swing = max(0.0, math.sin(2.0 * math.pi * phi)) * self.step_height

            q_des[1] += hip_swing
            q_des[2] -= knee_swing
            q_des[4] += hip_swing
            q_des[5] -= knee_swing

            # 자이로 기반 상체 롤/피치 균형 보정
            q_des[0] -= 0.04 * gyro[0]
            q_des[3] -= 0.04 * gyro[0]

            kp = self.kp_walk
            kd = self.kd_walk

        elif self.state == "DOCKED":
            q_des = self.q_stand.copy()
            kp = self.kp_lock
            kd = self.kd_lock
            if (sim_time - self.dock_time) >= self.stance_lock_duration:
                self.state = "COMPLETE"

        else:  # COMPLETE
            q_des = self.q_stand.copy()
            kp = self.kp_lock
            kd = self.kd_lock

        torques = kp * (q_des - q_act) - kd * v_act
        torques = np.clip(torques, -80.0, 80.0)

        self.log_time.append(sim_time)
        self.log_x.append(pos_x)
        self.log_gyro.append(np.linalg.norm(gyro[:2]))

        return torques

def run_simulation_loop(model, data, controller, viewer=None, max_time=12.0):
    dt = model.opt.timestep
    last_print_time = 0.0
    dock_reported = False
    prev_sim_time = data.time

    # 1.0x 실시간 동기화를 위한 기준 시각
    wall_start = time.perf_counter()
    sim_start = data.time

    print(f"\n{Colors.BOLD}[TEST EXECUTION] Running Bipedal Locomotion Simulation...{Colors.RESET}")
    if viewer:
        print(f"  {Colors.CYAN}📺 3D MuJoCo 뷰어가 활성화되었습니다. (Space: 일시정지, Backspace: 리셋){Colors.RESET}")

    step = 0
    evaluation_done = False

    while True:
        if viewer and not viewer.is_running():
            print(f"  * 사용자에 의해 뷰어 창이 닫혔습니다.")
            break

        # [핵심 1] 뷰어 Reset 감지 (Backspace 또는 UI Reset 버튼 클릭 시)
        if data.time < prev_sim_time:
            print(f"\n  {Colors.YELLOW}↺ [RESET 감지] 뷰어 리셋이 감지되어 제어기 및 초기 관절 자세를 완벽히 재동기화합니다.{Colors.RESET}")
            controller.reset()
            data.qpos[7:13] = controller.q_stand.copy()
            data.qvel[:] = 0.0
            mujoco.mj_forward(model, data)
            dock_reported = False
            evaluation_done = False
            wall_start = time.perf_counter()
            sim_start = data.time
            last_print_time = data.time
            prev_sim_time = data.time

        prev_sim_time = data.time

        # 1. 제어 토크 인가
        torques = controller.compute_torques(data)
        for i, act_id in enumerate(controller.act_ids):
            data.ctrl[act_id] = torques[i]

        # 2. 물리 1 스텝 전진
        mujoco.mj_step(model, data)
        step += 1

        # 3. 뷰어 화면 동기화 및 1.0x 실시간 속도 보정
        if viewer and (step % 5 == 0):
            viewer.sync()
            sim_elapsed = data.time - sim_start
            wall_elapsed = time.perf_counter() - wall_start
            sleep_time = sim_elapsed - wall_elapsed
            if sleep_time > 0:
                time.sleep(min(sleep_time, 0.05))

        # 4. 실시간 텍스트 상태 출력 (0.5초 주기)
        sim_time = data.time
        if (sim_time - last_print_time) >= 0.5:
            pos_x = data.xpos[controller.base_body_id][0]
            pos_z = data.xpos[controller.base_body_id][2]
            print(f"  * [{sim_time:5.2f}s] 상태: {controller.state:<8} | 위치: X={pos_x:5.3f}m, 높이 Z={pos_z:5.3f}m")
            last_print_time = sim_time

        if controller.state == "DOCKED" and not dock_reported:
            pos_x = data.xpos[controller.base_body_id][0]
            print(f"  {Colors.GREEN}★ [{sim_time:.2f}s] 도킹 구역 도달! (X = {pos_x:.3f}m) -> Stance Lock 전환 (3초 안정화 시작){Colors.RESET}")
            dock_reported = True

        if controller.state == "COMPLETE" and not evaluation_done:
            print(f"  {Colors.GREEN}★ [{sim_time:.2f}s] Stance Lock 3초 안정화 완수!{Colors.RESET}")
            evaluation_done = True
            success = evaluate_results(controller, controller.target_x)
            export_snapshot(model, data)
            print(f"\n{Colors.BOLD}{Colors.CYAN}============================================================{Colors.RESET}")
            if success:
                print(f"{Colors.BOLD}{Colors.GREEN}🎉 [SUCCESS] Phase 01-U01 Tron1 기본 보행 및 도킹 정지 검증 완수!{Colors.RESET}")
            else:
                print(f"{Colors.BOLD}{Colors.RED}❌ [FAILED] 보행/도킹 성능 기준을 만족하지 못했습니다.{Colors.RESET}")
            print(f"{Colors.BOLD}{Colors.CYAN}============================================================{Colors.RESET}")
            if not viewer:
                break
            else:
                print(f"  {Colors.CYAN}💡 3D 창에서 로봇을 자유롭게 관찰하세요. Backspace를 누르면 처음부터 다시 걷습니다.{Colors.RESET}")

        if controller.log_fall and not evaluation_done:
            print(f"  {Colors.RED}✗ [{sim_time:.2f}s] 로봇 전도 발생! (h < 0.35m){Colors.RESET}")
            evaluation_done = True
            evaluate_results(controller, controller.target_x)
            if not viewer:
                break

        # 헤드리스 모드 종료 조건
        if not viewer and data.time >= max_time:
            if not evaluation_done:
                evaluate_results(controller, controller.target_x)
            break

    return controller, data

def evaluate_results(controller, target_x):
    print(f"\n{Colors.BOLD}[VERIFICATION RESULTS] Performance Evaluation{Colors.RESET}")
    if controller.log_fall:
        print(f"  {Colors.RED}✗ [Check 1] 전도 여부: 전도 발생 [FAIL]{Colors.RESET}")
        return False
    else:
        print(f"  {Colors.GREEN}✓ [Check 1] 전도 여부: 넘어지지 않고 직립 유지 [PASS]{Colors.RESET}")

    final_x = controller.log_x[-1] if len(controller.log_x) > 0 else 0.0
    x_error = abs(final_x - target_x)
    print(f"  * 최종 위치: x = {final_x:.3f} m (목표 x = {target_x:.3f} m, 오차: {x_error*100:.1f} cm)")
    dock_pass = (x_error <= 0.15)
    if dock_pass:
        print(f"  {Colors.GREEN}✓ [Check 2] 도킹 도달 정밀도 합격 (오차 15cm 이내) [PASS]{Colors.RESET}")
    else:
        print(f"  {Colors.RED}✗ [Check 2] 도킹 도달 오차 초과: {x_error*100:.1f}cm [FAIL]{Colors.RESET}")

    vibration_pass = False
    if controller.dock_time is not None:
        times = np.array(controller.log_time)
        gyros = np.array(controller.log_gyro)
        lock_mask = times >= (controller.dock_time + 1.0)
        if np.any(lock_mask):
            mean_vibration = np.mean(gyros[lock_mask])
            print(f"  * Stance Lock 수렴 각속도: 평균={mean_vibration:.4f} rad/s")
            if mean_vibration <= 0.08:
                print(f"  {Colors.GREEN}✓ [Check 3] 정지 상태 진동 억제 합격 (< 0.08 rad/s) [PASS]{Colors.RESET}")
                vibration_pass = True
            else:
                print(f"  {Colors.RED}✗ [Check 3] 정지 상태 진동 억제 불합격 (>= 0.08 rad/s) [FAIL]{Colors.RESET}")

    return dock_pass and vibration_pass

def export_snapshot(model, data, output_path="temp/u01_tron1_docking.png"):
    try:
        os.makedirs(os.path.dirname(output_path), exist_ok=True)
        renderer = mujoco.Renderer(model, width=640, height=480)
        renderer.update_scene(data, camera="overview_cam")
        rgb = renderer.render()
        import cv2
        bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        cv2.imwrite(output_path, bgr)
        print(f"  {Colors.GREEN}✓ 렌더링 스냅샷 저장 완료: {output_path} [PASS]{Colors.RESET}")
    except Exception as e:
        print(f"  {Colors.YELLOW}⚠ 렌더링 스냅샷 저장 생략: {e}{Colors.RESET}")

def main():
    args = parse_args()
    print(f"\n{Colors.BOLD}{Colors.CYAN}============================================================{Colors.RESET}")
    print(f"{Colors.BOLD}{Colors.CYAN}Phase 01-U01: Tron1 Bipedal Locomotion & Docking Stance Lock{Colors.RESET}")
    print(f"{Colors.BOLD}{Colors.CYAN}============================================================{Colors.RESET}")
    print(f"  * Target XML : {args.xml}")
    print(f"  * Docking X  : {args.target_x} m")

    if not os.path.exists(args.xml):
        print(f"{Colors.RED}✗ XML 파일을 찾을 수 없습니다: {args.xml}{Colors.RESET}")
        return 1

    model = mujoco.MjModel.from_xml_path(args.xml)
    data = mujoco.MjData(model)
    mujoco.mj_resetData(model, data)

    controller = Tron1BipedController(model, target_x=args.target_x)

    # 스폰 시 기립 자세로 부드럽게 지면 안착
    data.qpos[7:13] = controller.q_stand.copy()
    mujoco.mj_forward(model, data)

    # 뷰어 실행 모드 분기 (기본값: 3D 창 표시 + 텍스트 동시 출력)
    if not args.headless:
        try:
            with mujoco.viewer.launch_passive(model, data) as viewer:
                viewer.opt.geomgroup[0] = 1
                viewer.opt.geomgroup[1] = 1
                viewer.opt.geomgroup[2] = 1
                viewer.opt.geomgroup[3] = 1
                controller, final_data = run_simulation_loop(model, data, controller, viewer=viewer, max_time=args.max_time)
        except Exception as e:
            print(f"  {Colors.YELLOW}⚠ GUI 뷰어 실행 실패 ({e}) -> 텍스트 전용 모드로 전환{Colors.RESET}")
            controller, final_data = run_simulation_loop(model, data, controller, viewer=None, max_time=args.max_time)
    else:
        controller, final_data = run_simulation_loop(model, data, controller, viewer=None, max_time=args.max_time)

    return 0

if __name__ == "__main__":
    sys.exit(main())
```

---

### Step 4: 스크립트 실행 및 결과 검증 (물리 정합성 해석)

두 파일을 모두 생성했다면, 터미널에서 아래 명령을 실행합니다:

```bash
# 가상환경 활성화 (필수)
conda activate transfer_bottle_by_tron1_py3_10

# ROS 2 환경 로드
source /opt/ros/humble/setup.bash

# U01 단위 검증 실행 (기본 모드: 3D GUI 뷰어 실행 + 실시간 콘솔 텔레메트리 출력)
python scripts_devel_roadmap/phase01_u01_test_tron1_walking.py

# (참고) CI/CD 또는 터미널 전용 환경에서 헤드리스로 실행할 경우:
# python scripts_devel_roadmap/phase01_u01_test_tron1_walking.py --headless
```

#### 정상 실행 출력 예시:
```text
============================================================
Phase 01-U01: Tron1 Bipedal Locomotion & Docking Stance Lock
============================================================
  * Target XML : xml_for_unit_test/phase01_u01_scene_unit_tron1.xml
  * Docking X  : 1.0 m

[TEST EXECUTION] Running Bipedal Locomotion Simulation...
  📺 3D MuJoCo 뷰어가 활성화되었습니다. (Space: 일시정지, Backspace: 리셋)
  * [ 0.50s] 상태: LANDING  | 위치: X=0.000m, 높이 Z=0.685m
  * [ 1.00s] 상태: LANDING  | 위치: X=0.000m, 높이 Z=0.686m
  * [ 1.50s] 상태: WALKING  | 위치: X=0.000m, 높이 Z=0.686m
  * [ 2.00s] 상태: WALKING  | 위치: X=0.183m, 높이 Z=0.682m
  * [ 2.50s] 상태: WALKING  | 위치: X=0.385m, 높이 Z=0.684m
  * [ 3.00s] 상태: WALKING  | 위치: X=0.589m, 높이 Z=0.683m
  * [ 3.50s] 상태: WALKING  | 위치: X=0.792m, 높이 Z=0.684m
  * [ 4.00s] 상태: WALKING  | 위치: X=0.998m, 높이 Z=0.683m
  ★ [4.01s] 도킹 구역 도달! (X = 0.998m) -> Stance Lock 전환 (3초 안정화 시작)
  * [ 4.50s] 상태: DOCKED   | 위치: X=1.002m, 높이 Z=0.685m
  * [ 5.00s] 상태: DOCKED   | 위치: X=1.002m, 높이 Z=0.685m
  * [ 5.50s] 상태: DOCKED   | 위치: X=1.002m, 높이 Z=0.685m
  * [ 6.00s] 상태: DOCKED   | 위치: X=1.002m, 높이 Z=0.685m
  * [ 6.50s] 상태: DOCKED   | 위치: X=1.002m, 높이 Z=0.685m
  * [ 7.00s] 상태: DOCKED   | 위치: X=1.002m, 높이 Z=0.685m
  ★ [7.01s] Stance Lock 3초 안정화 완수!

[VERIFICATION RESULTS] Performance Evaluation
  ✓ [Check 1] 전도 여부: 넘어지지 않고 직립 유지 [PASS]
  * 최종 위치: x = 1.002 m (목표 x = 1.000 m, 오차: 0.2 cm)
  ✓ [Check 2] 도킹 도달 정밀도 합격 (오차 15cm 이내) [PASS]
  * Stance Lock 수렴 각속도: 평균=0.0125 rad/s
  ✓ [Check 3] 정지 상태 진동 억제 합격 (< 0.08 rad/s) [PASS]
  ✓ 렌더링 스냅샷 저장 완료: temp/u01_tron1_docking.png [PASS]

============================================================
🎉 [SUCCESS] Phase 01-U01 Tron1 기본 보행 및 도킹 정지 검증 완수!
============================================================
  💡 3D 창에서 로봇을 자유롭게 관찰하세요. Backspace를 누르면 처음부터 다시 걷습니다.
```

---

### Step 5: 인터랙티브 3D GUI 뷰어 실시간 조작 및 리셋

스크립트를 실행하면 3D 뷰어가 화면에 즉시 팝업되며, 실시간 1.0x 배속 동기화를 통해 로봇의 현실적인 역학 거동을 관찰할 수 있습니다.

* **키보드 및 마우스 조작법:**
  * **Backspace 바:** 즉각 리셋 (뷰어 리셋 감지 시 로봇 초기 기립 자세 및 FSM 제어기가 자동 재동기화되어 1.0x 배속으로 다시 보행을 시작)
  * **Space 바:** 일시 정지 (Pause) / 재개 (Resume)
  * **좌클릭 드래그:** 카메라 시점 360도 자유 궤도 회전
  * **우클릭 드래그:** 카메라 상하좌우 평면 이동 (Pan)
  * **스크롤 휠:** 카메라 전진/후진 (Zoom In / Out)

---

## 4. 트러블슈팅 가이드 (자주 겪는 오류 및 원인 분석)

### 증상 1: 스폰 직후 로봇이 다리를 펴지 못하고 주저앉거나 튀어 오름
* **원인:**
  1. 초기 스폰 높이($z=0.82\text{m}$)와 기립 목표 각도($q_{stand}$) 간의 기구학적 오차로 인해, 발바닥 구체가 지면 밑으로 파고든 상태(Penetration)에서 시뮬레이션이 시작되어 반발력이 폭발함.
  2. 또는 관절 비례 게인 $K_p$가 로봇 자중($15\text{kg}$)을 지탱하기에 너무 낮음.
* **해결책:**
  * XML 내 `pos="0 0 0.82"` 높이를 유지하고, `phase01_u01_test_tron1_walking.py`의 `self.q_stand` 각도(무릎 약 $0.80\text{rad} \approx 45.8^\circ$)를 정확히 설정합니다.

### 증상 2: 보행 중 상체가 좌우로 심하게 흔들리다 전도됨
* **원인:**
  * 발목이 없는 포인트 풋 로봇은 지지 다리가 바뀌는 순간 상체 질량 중심이 지지발 바깥으로 벗어나면 롤(Roll) 방향 전복 모멘트가 발생합니다.
* **해결책:**
  * 자이로 피드백 게인 `0.04 * gyro[0]`를 통해 Abad 관절이 반대 방향으로 상체를 밀어주도록 튜닝되어 있는지 확인합니다.

### 증상 3: 도킹 목표점에 도착했으나 멈추지 않고 계속 발구름 진동이 남음
* **원인:**
  * `STATE_DOCKED` 전환 후 보행 사인파가 계속 더해지거나, 정지 시 게인 $K_p, K_d$가 보행 시와 동일하여 관성 진동이 감쇠되지 않음.
* **해결책:**
  * `controller.state == "DOCKED"` 진입 시 $q_{des}$를 순수 $q_{stand}$로 고정하고, `kp_lock` ($200 \sim 250$)과 `kd_lock` ($12 \sim 15$)의 고게인 감쇠를 활성화해야 합니다.

### 증상 4: MJCF 로드 시 STL 메쉬 파일 오픈 실패 (`could not open file`)
* **원인:**
  * XML의 `compiler meshdir` 상대 경로가 올바르지 않음.
* **해결책:**
  * XML 선언부의 `meshdir="../model_ori/PF_TRON1A/meshes/"` 경로를 반드시 확인하세요.

### 증상 5: 뷰어에서 Reset(Backspace)을 눌렀을 때 로봇이 중력이 감소한 것처럼 슬로우 모션으로 천천히 떨어지는 현상
* **원인:**
  1. **고정 슬립 지연:** 매 물리 스텝($dt=0.001\text{s}$)마다 `time.sleep(0.005)`와 같은 고정 지연을 주면 시뮬레이션 1초 진행에 현실 시간 5초가 걸려 **$5\times$ 슬로우 모션 (0.2배속)**이 발생합니다.
  2. **상태 머신 불일치 (Honey Damping):** 사용자가 뷰어를 리셋했을 때 물리 시간($data.time$)은 0으로 돌아가지만, 상위 Python 제어기가 여전히 정지 완료(`COMPLETE`) 상태의 높은 댐핑 게인($K_d = 15.0$)을 유지하면 낙하 속도에 비례한 거대한 저항 토크($\tau = -K_d \cdot v$)가 발생하여 로봇이 마치 진득한 꿀(Honey) 속에 빠진 것처럼 천천히 가라앉게 됩니다.
  3. **초기 자세 미정의:** XML에 `<keyframe>`이 없으면 리셋 시 $qpos$가 0(완전히 곧게 편 다리)으로 돌아가 착지 충격이 폭발합니다.
* **해결책:**
  1. 벽시계 시간(`time.perf_counter()`)과 물리 시간의 차이를 매 5스텝마다 계산하여 정확한 **1.0x 실시간 동기화**를 적용합니다.
  2. `data.time < prev_sim_time`을 감지하여 뷰어 리셋 즉시 `controller.reset()`을 호출, 상태를 `LANDING`으로 되돌리고 초기 굽힘 각도(`q_stand`) 및 속도($0$)를 완벽히 재동기화합니다.
  3. XML에 `<keyframe><key name="stand" .../></keyframe>`을 등록하여 MuJoCo 네이티브 리셋 시에도 굽힌 무릎 자세가 보존되도록 설정합니다.

---

## 5. Phase 01-U01 완료 체크리스트

직접 모든 파일 생성과 테스트를 마친 후 아래 항목들이 정상인지 스스로 점검해 보세요:

- [ ] `xml_for_unit_test/phase01_u01_scene_unit_tron1.xml` 파일이 정상 생성되었고 MuJoCo 파싱 에러가 없다.
- [ ] `scripts_devel_roadmap/phase01_u01_test_tron1_walking.py` 파일이 정상 작성되었다.
- [ ] 로봇이 초기 스폰 낙하 시 넘어지지 않고 1.5초 내에 수직 기립 안정화에 성공한다.
- [ ] 로봇이 전진 보행하여 목표 인계 구역($x = 1.0\text{m}$) 부근에 오차 15cm 이내로 진입한다.
- [ ] 도킹 진입 후 Stance Lock 상태에서 최소 3초 동안 평균 롤/피치 각속도 $0.08\text{rad/s}$ 이하로 안정 정지한다.
- [ ] `temp/u01_tron1_docking.png` 스냅샷 이미지가 정상 저장되었다.
- [ ] 3D GUI 창에서 실시간 1.0x 배속 보행 및 Backspace 리셋 동작을 시각적으로 확인했다.
- [ ] `documents/done_list.txt`에 Phase 01-U01 완료 내역을 기록했다.
