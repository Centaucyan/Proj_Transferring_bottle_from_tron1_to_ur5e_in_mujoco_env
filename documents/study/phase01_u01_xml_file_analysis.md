# [기술 분석 보고서] Phase 01-U01 Tron1 MuJoCo 시뮬레이션 환경 XML 심층 분석

* **대상 파일**: [`unit_test_models/phase01_u01_scene_unit_tron1.xml`](file:///media/korit/4AD6F9D1D6F9BCEF/Tae_ws/Project/Proj_Transferring_bottle_from_tron1_to_ur5e_in_mujoco_env/unit_test_models/phase01_u01_scene_unit_tron1.xml)
* **연동 검증 스크립트**: [`scripts_devel_roadmap/phase01_u01_test_tron1_walking.py`](file:///media/korit/4AD6F9D1D6F9BCEF/Tae_ws/Project/Proj_Transferring_bottle_from_tron1_to_ur5e_in_mujoco_env/scripts_devel_roadmap/phase01_u01_test_tron1_walking.py)
* **참조 모델**: [`model_ori/PF_TRON1A/xml/robot.xml`](file:///media/korit/4AD6F9D1D6F9BCEF/Tae_ws/Project/Proj_Transferring_bottle_from_tron1_to_ur5e_in_mujoco_env/model_ori/PF_TRON1A/xml/robot.xml)
* **작성 목적**: LimX Dynamics Tron1 2족 보행 로봇의 단독 보행 및 도킹 정지 환경의 물리 엔진 설정, 기구학 체인, 액추에이터/센서 시스템, 그리고 하드웨어 정합성을 단계별로 완벽히 분석 및 정리.

---

## 목차
1. [1단계: 컴파일러 및 메모리 버퍼 설정 (`<compiler>`, `<size>`)](#1단계-컴파일러-및-메모리-버퍼-설정-compiler-size)
2. [2단계: 전역 물리 옵션 (`<option>`)](#2단계-전역-물리-옵션-option)
3. [3단계: 뷰어 및 시각화 디버깅 옵션 (`<visual>`)](#3단계-뷰어-및-시각화-디버깅-옵션-visual)
4. [4단계: 공통 에셋 및 재질 정의 (`<asset>`)](#4단계-공통-에셋-및-재질-정의-asset)
5. [5단계: 기본 클래스 정의 (`<default>`) 및 충돌체 분리](#5단계-기본-클래스-정의-default-및-충돌체-분리)
6. [6단계: 월드 바디 및 Tron1 로봇 본체 기구학 (`<worldbody>`)](#6단계-월드-바디-및-tron1-로봇-본체-기구학-worldbody)
7. [7단계: 액추에이터 정의 (`<actuator>`) 및 하드웨어 스펙 검증](#7단계-액추에이터-정의-actuator-및-하드웨어-스펙-검증)
8. [8단계: 센서 시스템 정의 (`<sensor>`)](#8단계-센서-시스템-정의-sensor)
9. [9단계: 기본 기립 키프레임 (`<keyframe>`) 및 파이썬 제어기 연동 이슈](#9단계-기본-기립-키프레임-keyframe-및-파이썬-제어기-연동-이슈)

---

## 1단계: 컴파일러 및 메모리 버퍼 설정 (`<compiler>`, `<size>`)

```xml
<!-- 1. 컴파일러 설정: 각도 라디안, 메쉬 상대경로 바인딩 -->
<compiler angle="radian" coordinate="local" meshdir="../model_ori/PF_TRON1A/meshes/" autolimits="true"/>

<size njmax="1000" nconmax="200"/>
```

### 1.1 `<compiler>` 속성 해설
* **`angle="radian"`**: 모든 각도(관절 가동 범위, 오일러 회전 등) 단위를 도(degree) 대신 라디안($\text{rad}$)으로 통일합니다.
* **`coordinate="local"`**: 모든 자식 링크의 위치(`pos`) 및 회전을 부모 링크(Parent Body) 기준의 로컬 좌표계로 기술합니다.
* **`meshdir`**: 3D 외형 메쉬(STL) 파일들이 저장된 폴더 경로를 상대 경로로 바인딩합니다.
* **`autolimits="true"`**: 관절의 `range` 속성을 바탕으로 관절 한계 구속조건(Limit)을 자동 활성화합니다.

### 1.2 `<size>` 태그 미지정 시 동작 방식
* **생략 시**: MuJoCo 컴파일러가 모델의 바디, 지오메트리, 조인트 수를 분석하여 휴리스틱(Heuristic) 기반 기본값을 자동 할당합니다.
* **미지정 시의 위험성 (버퍼 오버플로우)**:
  * 로봇이 심하게 넘어지거나 충돌이 폭증하여 실제 접촉 수가 자동 추정치를 초과하면 `mjWARN_CONTACTFULL` 또는 `mjWARN_CNSTRFULL` 경고가 발생합니다.
  * 초과된 접촉 계산이 통째로 누락되어 **로봇 발이나 몸체가 바닥을 뚫고 가라앉는 터널링 현상**이 일어납니다.

### 1.3 `nconmax="200"`과 `njmax="1000"`의 엔지니어링 산출 근거
* **`nconmax` (최대 동시 접촉점 수)**:
  * 정상 보행 시: 양발 구체 접촉점 $1 + 1 = 2$개.
  * 최악의 전도(넘어짐) 시:
    * 상체 박스(Box) 면 접촉: 최대 4개
    * 골반, 허벅지, 종아리 실린더(Cylinder): $2\text{개} \times 3\text{쌍} = 6\sim 18$개
    * 발끝 구체(Sphere): 2개
    * 바닥 접촉 총합: 약 26개 안팎
    * 다리 간 자체 충돌(Self-Collision): 약 $5\sim 10$개
    * $\rightarrow$ 최악의 참사 상황에서도 순간 접촉점은 약 **$30\sim 40$개** 수준.
    * 이에 약 5배의 안전 여유(Safety Margin)를 두어 **`200`**으로 결정.
* **`njmax` (최대 스칼라 제약조건 수)**:
  * 이 파일은 3차원 마찰 접촉(`condim="3"`)을 사용하므로, **접촉점 1개당 3개의 제약 수식**(수직항력 1 + 직교 마찰력 2)이 파생됩니다.
  * 최대 접촉 200개 $\times 3 = 600$개.
  * 관절 가동 범위 한계 제약(최대 6개) 추가 시 약 606개.
  * 계산 여유 및 정수 반올림을 고려해 **`1000`**으로 결정.
* **버퍼를 과도하게 크게(예: 10만) 잡지 않는 이유**:
  * 시작 시 C언어 정적 메모리로 통째 할당되므로 RAM 낭비 및 CPU 캐시 미스가 발생함.
  * 특히 **수천 개 환경을 병렬로 띄우는 강화학습(RL) 시 메모리 폭발(OOM)**의 원인이 됨.

---

## 2단계: 전역 물리 옵션 (`<option>`)

```xml
<!-- 2. 전역 물리 옵션 (U00 표준 준수) -->
<option timestep="0.001" gravity="0 0 -9.81" integrator="implicitfast" cone="elliptic">
  <flag contact="enable"/>
</option>
```

### 2.1 주요 설정 해설
* **`timestep="0.001"` ($1\,\text{ms}$, $1000\,\text{Hz}$)**:
  * 보행 로봇은 발 디딤 순간의 지면반력(Impact) 변화가 매우 가파르므로, $1\,\text{ms}$의 미세 시간 간격으로 계산해야 수치적 튕김이나 발산이 없습니다.
* **`gravity="0 0 -9.81"`**: $z$축 음의 방향으로 표준 중력 가속도 적용.
* **`<flag contact="enable"/>`**: 물체 간 접촉 충돌 계산 활성화.

### 2.2 적분기(Integrator) 3종 비교 및 `implicitfast` 선정 이유

| 적분기 | 연산 속도 | 수치 안정성 | 특징 및 보행 로봇 적합도 |
| :--- | :---: | :---: | :--- |
| **`Euler`** | ⚡ 매우 빠름 | 💥 충돌 시 취약 | 관절 댐핑이나 지면 충돌 시 수치 발산(`NaN`)으로 튕겨 나감 |
| **`implicit`** | 🐢 매우 느림 | 🛡️ 완벽 방어 | 전 비선형 도함수 행렬을 계산하여 극도로 안정적이나 너무 무거움 |
| **`implicitfast`** | ⚡ **매우 빠름** | 🛡️ **극도로 안정적** | **관절 댐핑/스프링 항만 해석적 암시적 적분 수행. Euler의 속도와 Implicit의 안정성을 모두 확보 (이족보행 표준)** |

### 2.3 마찰 원뿔 모델 (`cone="elliptic"`)과 마찰력 결정 메커니즘
* `cone="elliptic"` 자체는 마찰력의 크기를 결정하는 것이 아니라, **360도 전 방향으로 균일한 원형 마찰 한계 법칙**을 적용하는 수학적 틀입니다. (기본값인 `pyramidal`의 대각선 마찰 왜곡 $\sqrt{2}$배 현상 제거).
* 실제 마찰력의 세기는 개별 충돌체의 `friction` 속성에서 결정됩니다:
  * 바닥: `friction="1.0 0.005 0.0001"`
  * 발끝: `friction="1.2 0.005 0.0001"` (미끄러짐 방지를 위한 고접지 세팅)
  * MuJoCo 충돌 규칙에 따라 두 접촉체 중 최댓값인 **$\mu = 1.2$**가 실제 접촉면에 적용됩니다.

---

## 3단계: 뷰어 및 시각화 디버깅 옵션 (`<visual>`)

```xml
<!-- 3. 뷰어 및 시각화 기본 설정 -->
<visual>
  <global offwidth="640" offheight="480" azimuth="135" elevation="-20"/>
  <quality shadowsize="2048" offsamples="4"/>
  <rgba com="0.2 0.8 0.2 0.6" contactforce="0.8 0.2 0.2 0.8"/>
  <scale com="0.1" forcewidth="0.04" contactwidth="0.08"/>
</visual>
```

* **카메라 기본 앵글**: 방위각 $135^\circ$, 앙각 $-20^\circ$의 대각선 쿼터뷰로 시작하여 로봇의 보행 및 발 디딤을 관찰하기 최적화.
* **렌더링 품질**: 그림자 맵 $2048 \times 2048$, MSAA 4배 안티앨리어싱.
* **보행 로봇 필수 디버깅 도구**:
  * **`com` (질량중심)**: 로봇의 전체 무게중심 위치에 **직경 $20\,\text{cm}$의 연두색 반투명 구체**를 실시간 렌더링.
  * **`contactforce` (지면반력)**: 발바닥이 땅을 누를 때 발생하는 지면반력(GRF) 벡터를 **붉은색 화살표**로 실시간 표시.

---

## 4단계: 공통 에셋 및 재질 정의 (`<asset>`)

```xml
<texture type="skybox" builtin="gradient" rgb1="0.3 0.5 0.7" rgb2="0 0 0" width="512" height="512"/>
<texture name="texplane" type="2d" builtin="checker" rgb1="0.2 0.25 0.3" rgb2="0.3 0.35 0.4"
         width="512" height="512" mark="cross" markrgb="0.8 0.8 0.8"/>
<material name="matplane" texture="texplane" texrepeat="5 5" texuniform="true" reflectance="0.1"/>

<!-- Tron1 3D CAD 메쉬 9개 로드 -->
<mesh name="base_Link" file="base_Link.STL"/> ...
```

* **$1\,\text{m} \times 1\,\text{m}$ 격자 바닥 (`texplane` & `matplane`)**: 십자 눈금($+$)이 표시된 $1\,\text{m}$ 체크 패턴으로, 로봇의 전진 이동 거리와 보폭을 육안으로 직관적으로 측정하는 '시각적 줄자' 역할 수행.
* **3D 외형 메쉬**: 상체 및 좌우 다리 링크 총 9개의 CAD STL 메쉬 로드.
* **재질**: 로봇 외장 메탈릭 화이트(`robot_body_mat`), 모터 구동부 흑회색(`robot_dark_mat`), 도킹 목표 반투명 초록색(`target_marker_mat`).

---

## 5단계: 기본 클래스 정의 (`<default>`) 및 충돌체 분리

```xml
<default>
  <default class="visual">
    <geom contype="0" conaffinity="0" group="2" type="mesh"/>
  </default>
  <default class="collision">
    <geom contype="1" conaffinity="1" condim="3" group="3" friction="1.0 0.005 0.0001"/>
  </default>
  <joint armature="0.02" damping="0.05" limited="true"/>
</default>
```

### 5.1 시각(`visual`)과 충돌(`collision`)의 분리 이유
* **연산 부하 최적화**: 수만 개의 폴리곤을 가진 3D STL 메쉬는 충돌 계산을 끄고(`contype="0" conaffinity="0"`, Group 2) 순수 렌더링에만 사용합니다.
* **단순 형상 충돌체**: 실제 충돌 계산은 구체(Sphere), 실린더(Cylinder), 박스(Box) 등의 기본 도형(`contype="1" conaffinity="1"`, Group 3)으로 초고속 계산을 수행합니다.
* **충돌체가 없다면?**: 충돌 속성이 켜진 지오메트리가 전혀 없다면 로봇은 바닥을 뚫고 지하로 영원히 추락합니다.

### 5.2 `robot.xml`과 `phase01`의 머리 형태 차이 원인 규명
* **`robot.xml`**: 충돌용 네모 상자(`base_collision`)에 `rgba="0 0 1 0"`(알파값 0)을 주어 **강제로 투명하게 숨김**. 따라서 메쉬만 보여 날렵해 보임.
* **`phase01`**: 충돌체에 투명도를 주지 않고 **`group="3"` 레이어로 분리**.
  * 뷰어에서 Group 3이 켜져 있으면 상체 메쉬 바깥으로 충돌용 직육면체 상자(`base_col`)가 튀어나와 머리가 네모나게 보임.
  * **해결법**: 뷰어에서 **키보드 `3`번**을 누르면 충돌체 레이어가 꺼지면서 날렵한 원래 메쉬 외형만 나타남.

### 5.3 관절 기본 설정
* `armature="0.02"`: 모터 로터 회전 관성을 모사하여 제어 시 고주파 잔떨림 방지.
* `damping="0.05"`: 점성 감쇠를 주어 관절의 잔여 진동 흡수.
* `limited="true"`: 가동 범위 한계 강제.

---

## 6단계: 월드 바디 및 Tron1 로봇 본체 기구학 (`<worldbody>`)

### 6.1 환경 요소
* **도킹 목표 마커 (`docking_target_marker`)**: $x=1.0\,\text{m}$, $y=0.0\,\text{m}$ 지점에 반지름 $0.25\,\text{m}$의 반투명 원반으로 배치 (로봇의 도착 목표 지점).
* **카메라**: 전체 조망용 `overview_cam` 및 로봇 CoM을 따라다니는 `track_cam` (`mode="trackcom"`).

### 6.2 Tron1 로봇 6-DOF 기구학 구조
* **스폰 위치**: `pos="0 0 0.82"` (다리를 살짝 굽혀 발끝이 지면에 정렬되는 높이).
* **`freejoint` (플로팅 베이스)**: 3차원 공간을 자유롭게 이동 및 회전하는 6자유도 자유 관절.
* **상체 질량**: $9.595\,\text{kg}$ 및 상체 중심에 `imu_site` 배치.
* **다리 기구학 체인 (한쪽 다리당 3-DOF, 총 6-DOF)**:

| 부위 | 관절 이름 | 회전축 (`axis`) | 가동 범위 (`range`) | 기구학적 역할 |
| :--- | :--- | :---: | :---: | :--- |
| **Left Abad** | `abad_L_Joint` | X축 ($1, 0, 0$) | $[-0.38, 1.39]\,\text{rad}$ | 롤(Roll) 방향 다리 벌림/모음 (외전/내전) |
| **Left Hip** | `hip_L_Joint` | Y축 ($0, 1, 0$) | $[-1.01, 1.39]\,\text{rad}$ | 피치(Pitch) 허벅지 앞뒤 굴곡/신전 |
| **Left Knee** | `knee_L_Joint` | Y축 역방향 ($0, -1, 0$) | $[-0.87, 1.36]\,\text{rad}$ | 피치(Pitch) 무릎 굴곡/신전 |
| **Left Foot** | `foot_L_col` | - | - | **반지름 0.032m 구체 포인트 풋** |
| **Right Leg**| `_R_Joint` | 대칭 회전축 | 대칭 가동 범위 | 좌측 다리와 부호가 반대인 대칭 구조 |

* **포인트 풋 (Point-Foot)**: 발목 모터가 없어 경량화 및 민첩성이 뛰어나지만, 정적 안정이 불가능하므로 제어기가 지속적으로 동적 균형(Dynamic Balance)을 유지해야 함.

---

## 7단계: 액추에이터 정의 (`<actuator>`) 및 하드웨어 스펙 검증

```xml
<actuator>
  <motor name="abad_L_motor" joint="abad_L_Joint" gear="1" ctrllimited="true" ctrlrange="-80 80"/>
  ... (총 6개 모터)
</actuator>
```

### 7.1 직접 토크 제어 (`<motor>`)의 필요성
* 위치 제어(`<position>`)와 달리 제어기가 각 관절의 출력 토크($\tau$, $\text{Nm}$)를 직접 입력합니다.
* 발 디딤 충격을 흡수하는 임피던스 제어 및 최신 MPC/강화학습 제어의 표준 방식입니다.

### 7.2 Tron1 공식 홈페이지 하드웨어 스펙 대조 분석

| 항목 | 공식 홈페이지 스펙 시트 | 현재 XML 설정값 | 분석 및 정합성 평가 |
| :--- | :---: | :---: | :--- |
| **정격 토크 (Rated)** | **$30\,\text{N}\cdot\text{m}$** | 미지정 | 연속 허용 토크 한계 |
| **피크 토크 (Peak)** | **$60\,\text{N}\cdot\text{m}$** | **$80\,\text{N}\cdot\text{m}$** (`ctrlrange="-80 80"`) | ⚠️ **실제 하드웨어 대비 약 33% 오버스펙** |
| **최대 속도 (Speed)** | **$15\,\text{rad/s}$** | 제한 없음 | 고속 회전 시 주의 필요 |

* **Sim-to-Real Gap 위험성**: 시뮬레이션에서 $70\,\text{N}\cdot\text{m}$를 사용하여 걷도록 학습된 정책을 실제 Tron1 로봇에 넣으면, 하드웨어 모터 드라이버가 $60\,\text{N}\cdot\text{m}$에서 토크를 잘라버려(Saturation) 로봇이 주저앉아 넘어지는 문제가 발생합니다.
* **권장 수정 방향**: 피크 한계를 `ctrlrange="-60 60"`으로 수정하고, 일반 보행 시에는 정격인 $30\,\text{N}\cdot\text{m}$ 이내를 유지하도록 제어 페널티를 부여해야 합니다.

---

## 8단계: 센서 시스템 정의 (`<sensor>`)

```xml
<sensor>
  <!-- 1. 상체 IMU 센서군 (10개 데이터) -->
  <framequat name="imu_quat" objtype="site" objname="imu_site"/>
  <gyro name="imu_gyro" site="imu_site"/>
  <accelerometer name="imu_acc" site="imu_site"/>

  <!-- 2. 관절 각도 엔코더 (6개 데이터) -->
  <jointpos name="pos_abad_L" joint="abad_L_Joint"/> ...
  <!-- 3. 관절 회전 속도 센서 (6개 데이터) -->
  <jointvel name="vel_abad_L" joint="abad_L_Joint"/> ...
</sensor>
```

### 8.1 22차원 상태 관측(Observation) 벡터 구성
파이썬에서 `data.sensordata`를 읽으면 다음 총 22개의 부동소수점 데이터가 1차원 배열로 반환됩니다:

1. **상체 3D 자세 쿼터니언 (`imu_quat`)**: $q_w, q_x, q_y, q_z$ (4개)
2. **상체 3축 회전 각속도 (`imu_gyro`)**: $\omega_x, \omega_y, \omega_z$ (3개)
3. **상체 3축 선가속도 (`imu_acc`)**: $a_x, a_y, a_z$ (3개)
4. **6개 관절 현재 각도 (`jointpos`)**: $q_1 \sim q_6$ (6개)
5. **6개 관절 현재 각속도 (`jointvel`)**: $\dot{q}_1 \sim \dot{q}_6$ (6개)
* $\rightarrow$ **총 합계: $4 + 3 + 3 + 6 + 6 = \mathbf{22\text{차원}}$**

---

## 9단계: 기본 기립 키프레임 (`<keyframe>`) 및 파이썬 제어기 연동 이슈

```xml
<keyframe>
  <key name="stand" qpos="0 0 0.82 1 0 0 0 0.0 0.40 0.80 0.0 -0.40 -0.80"/>
</keyframe>
```

### 9.1 13개 `qpos` 값의 물리적 매핑
* `0 0 0.82`: 상체 루트 위치 $(x, y, z)$
* `1 0 0 0`: 상체 회전 쿼터니언 (수평 유지)
* `0.0 0.40 0.80`: 좌측 다리 관절 각도 (abad=0, hip=+0.40 rad, knee=+0.80 rad)
* `0.0 -0.40 -0.80`: 우측 다리 관절 각도 (대칭 배치)

### 9.2 XML 키프레임 수정 후 파이썬 실행 시 초기 자세가 변하지 않았던 원인 분석
사용자가 XML의 `stand` 키프레임을 수정했음에도 `phase01_u01_test_tron1_walking.py` 실행 시 첫 자세가 바뀌지 않았던 이유는 **파이썬 코드 내 2가지 요인** 때문입니다:

1. **파이썬 코드에서 XML 키프레임을 미호출**:
   * 스크립트 305번째 줄: `mujoco.mj_resetData(model, data)`를 호출하고 있어 XML의 키프레임을 불러오지 않음 (`mj_resetDataKeyframe` 미사용).
2. **파이썬 제어기 내부의 초기 자세 하드코딩 및 강제 덮어쓰기**:
   * 스크립트 56번째 줄: `self.q_stand = np.array([0.0, 0.40, 0.80, 0.0, -0.40, -0.80])`가 하드코딩됨.
   * 스크립트 310번째 줄: `data.qpos[7:13] = controller.q_stand.copy()`로 관절 각도를 강제 덮어씀.
   * 시작 후 첫 1.5초간 `LANDING` 상태에서 모터가 $Kp=150$의 강력한 토크로 이 하드코딩된 자세를 강제로 복원함.

### 9.3 파이썬 스크립트 연동 해결 코드
XML 키프레임 변경이 파이썬 실행에 즉각 반영되도록 하려면 스크립트의 `main()` 함수를 다음과 같이 수정합니다:

```python
# scripts_devel_roadmap/phase01_u01_test_tron1_walking.py 수정 예시
stand_key_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "stand")
if stand_key_id != -1:
    # 1. XML 키프레임으로 물리 데이터 초기화
    mujoco.mj_resetDataKeyframe(model, data, stand_key_id)
    # 2. 제어기 목표 기본 자세를 XML 키프레임 값으로 동기화
    controller.q_stand = model.key_qpos[stand_key_id][7:13].copy()
```
