# [PRD] MuJoCo 기반 Tron1 이족보행 로봇 - UR5e 로봇팔 물병 인계 및 Pick-and-Place 시스템
# (Transferring Bottle from Tron1 to UR5e in MuJoCo Environment)

* **문서 버전:** v1.0
* **작성일:** 2026-09-03
* **프로젝트명:** Proj_Transferring_bottle_from_tron1_to_ur5e_in_mujoco_env
* **대상 환경:** Ubuntu 22.04 LTS / ROS 2 Humble / MuJoCo 3.x / Conda (`transfer_bottle_by_tron1_py3_10`)

---

## 1. 프로젝트 개요 (Overview)

### 1.1. 배경 및 목적
스마트 물류 및 제조 환경에서 2족 보행 로봇과 매니퓰레이터 간의 협업(Mobile Manipulation) 기술의 실효성이 크게 증대되고 있습니다.  
본 프로젝트는 **MuJoCo 물리 시뮬레이터 환경**에서 **2족 보행 로봇(Tron1)**이 운반해 온 물병류를 스테이션의 **RGB-D 카메라(RealSense D435i)**가 감지하고 기하학적 중심점을 산출하며, **협동로봇(UR5e + Robotiq 2F-85)**이 인계받아 지정 구역에 자동 분류/배치(Pick-and-Place)하는 **멀티 로봇 협업 자동화 시스템**을 개발하고 검증하는 것을 목적으로 합니다.

### 1.2. 핵심 가치 및 목표
* **동적 환경 협업:** 2족 보행 로봇의 보행/정지 안정성과 로봇팔의 정밀 조작 간의 인터락 및 동기화 구현.
* **비동기 파이프라이닝 (Pipelining):** 로봇팔이 물품을 내려놓는(Place) 시간 동안 카메라가 다음 물품을 미리 스캔·계산하는 사이클 타임(Cycle Time) 최적화 구조 적용.
* **강건한 예외 처리:** 물품 낙하, 헛파지, 미끄러짐(Slip), 시야 가림(Occlusion) 등 실제 환경 수준의 에러 감지 및 복구 로직 구현.

---

## 2. 시스템 아키텍처 및 하드웨어 명세 (Hardware & System Specs)

### 2.1. 하드웨어 구성
| 장비명 | 모델/사양 | 역할 | 시뮬레이션 모델 자산 |
| :--- | :--- | :--- | :--- |
| **이족 보행 로봇** | LimX Dynamics Tron1 | 트레이에 병류를 싣고 인계 구역으로 보행 운반, 트레이 높이 조절 | `model_ori/PF_TRON1A` |
| **협동 로봇팔** | Universal Robots UR5e | 6-DOF 매니퓰레이터, 병류 Pick & Place 모션 수행 | `model_ori/universal_robots_ur5e` |
| **그리퍼** | Robotiq 2F-85 | 2핑거 평행 그리퍼, 병 파지 상태 및 미끄러짐 감지 | `model_ori/robotiq_2f85` |
| **비전 센서** | Intel RealSense D435i | 고정형 RGB-D 카메라, 로봇 감지 및 3D 기하학적 중심점 산출 | `model_ori/realsense_d435i` |
| **작업대** | Station Table | 로봇팔/카메라 거치 및 병류 배치 구역(Place Zone) 제공 | MuJoCo Geoms 정의 |

### 2.2. 소프트웨어 스택
* **OS / Runtime:** Ubuntu 22.04 LTS, Python 3.10.x (Conda 가상환경 `transfer_bottle_by_tron1_py3_10`)
* **미들웨어:** ROS 2 Humble Hawksbill
* **물리 시뮬레이터:** MuJoCo 3.x (with `mujoco-python-viewer`)
* **모션 플래닝:** MoveIt 2 (`moveit_py` / C++ MoveGroup)
* **비전 및 포인트클라우드:** OpenCV 4.x, Open3D, `sensor_msgs_py`
* **시스템 제어/오케스트레이션:** `py_trees` (Behavior Tree) 또는 ROS 2 Action 기반 FSM

---

## 3. 작업 공간 및 배치 레이아웃 (Workspace Layout)
> 참조: `documents/scenario/work_space_01.png`

* **Tron1 진입 동선:** 시작 위치(정북/북서/북동)에서 장애물 없이 인계 구역(남쪽)으로 직진/대각 접근.
* **인계 구역 (Handoff Zone):** Station Table 북측 바로 앞, UR5e 우측에 위치.
* **UR5e 기저부 (Base):** 인계 구역 좌측(서쪽)에 설치되어 인계 구역과 Table Place 구역을 회전(Yaw)으로 커버 (최대 작업 반경 850mm).
* **RGB-D 카메라 마운트:** Station Table 좌상단 모서리에 거치대(높이 20~30cm, Tilt-down 20~30°)를 두어 인계 구역 트레이를 대각선 상단에서 조준 (이미 놓인 병에 의한 차폐 방지).
* **Place 구역:** Station Table 중앙/우측 상면에 위치.

---

## 4. 기능 요구사항 (Functional Requirements)

### FR-1. Tron1 보행 및 도킹 제어
* **FR-1.1:** 트레이 위에 놓인 병류를 쓰러뜨리지 않고 시작 위치에서 인계 구역까지 보행 이동해야 함.
* **FR-1.2:** 인계 구역 도달 시 트레이 상단 높이를 Station Table 높이와 일치하도록 조절(Squat/Stance)해야 함.
* **FR-1.3:** 인계 구역에서 최소 3초 이상 발구름 진동을 최소화하며 정지 상태(Stance Lock)를 유지해야 함.
* **FR-1.4:** 최종 작업 완료 신호를 수신하면 트레이 높이를 원복하고 초기 위치로 복귀해야 함.

### FR-2. RealSense D435i 비전 인식 및 기하학적 중심점 추출
* **FR-2.1:** 인계 구역에 진입하여 3초간 정지한 Tron1을 감지하고 상태를 확인해야 함.
* **FR-2.2:** 트레이 영역 포인트 클라우드에서 바닥 평면(RANSAC)을 제거하고 각 병류를 클러스터링해야 함.
* **FR-2.3:** UR5e와 가장 가까운 병을 우선순위로 선정하여 **3D 기하학적 중심점(x, y, z)**을 산출 및 송신해야 함.
* **FR-2.4 (사전 스캔):** UR5e가 병을 집어 Table로 이동하는 동안 트레이를 재스캔하여 다음 병 중심점을 사전 연산하고 대기해야 함.

### FR-3. UR5e & Robotiq 2F-85 매니퓰레이션
* **FR-3.1:** 수신된 기하학적 중심점 좌표로 MoveIt 2를 통해 무충돌(Collision-free) 파지 궤적을 계획/실행해야 함.
* **FR-3.2 (파지 확인):** 그리퍼 파지 시 `손가락 간격 + 벌림 간격 ≈ 병 지름` 여부를 엔코더 피드백으로 검증(헛파지 방지)해야 함.
* **FR-3.3 (미끄러짐 확인):** 리프트 동작 시 파지력 유지 및 Slip 여부를 확인해야 함.
* **FR-3.4:** 파지된 병을 Station Table의 지정 Place 슬롯에 직립 상태로 배치해야 함.
* **FR-3.5:** 개별 병 배치 완료 시 D435i에 즉시 완료 신호를 송신해야 함.

### FR-4. 예외 상황 처리 및 안전 제어 (Exception Handling & Safety)
* **EH-1 (빈 트레이):** 초기 도착 시 병 개수 0개이면 즉시 Tron1 복귀 명령 및 알림 발생.
* **EH-2 (전도/겹침):** 직립 상태가 불안정한 병은 스킵하고, 안정적인 다른 병을 우선 파지.
* **EH-3 (파지 실패/슬립):** 헛파지 감지 시 즉시 안전 위치로 후퇴 후 1회 재시도, 최종 실패 시 해당 병 포기 및 트레이 재스캔.
* **EH-4 (작업 영역 초과):** IK 풀이 실패 시 동작을 중단하고 Tron1에 위치 미세 조정 요청.
* **EH-5 (하드웨어 인터락):** UR5e 동작 중 Tron1 IMU 이상(외란/흔들림) 감지 시 로봇팔 즉시 비상 정지(E-Stop).
* **EH-6 (통신 타임아웃):** 완료 메시지가 30초 이상 지연될 경우 데드락 방지를 위한 안전 정지 및 상태 질의.

---

## 5. 비기능 요구사항 (Non-Functional Requirements)

* **성능 (Cycle Time):** 병렬 사전 스캔(Pipelining)을 통해 개당 Pick & Place 사이클 타임을 순차 방식 대비 30% 이상 단축.
* **정밀도:** 비전 좌표계와 로봇팔 베이스 좌표계 간 캘리브레이션 오차 ±5mm 이내 유지.
* **시뮬레이션 충실도:** MuJoCo 3.x 환경에서 강체 접촉 마찰력, 메시 충돌체, 관절 제약 조건 충실 반영.
* **모듈성:** Tron1 제어 노드, D435i 비전 노드, UR5e 매니퓰레이션 노드, FSM 오케스트레이터 노드로 분리 구현하여 독립 테스트 가능.

---

## 6. 개발 단계 및 마일스톤 (Milestones)

1. **M1. 시뮬레이션 환경 구축 (Scene Integration):**
   * Tron1, UR5e, 2F-85, D435i, Station Table을 통합한 MuJoCo `scene.xml` 구성.
2. **M2. ROS 2 - MuJoCo 브리지 및 단위 제어:**
   * 조인트 상태/커맨드 통신, 카메라 이미지 토픽 퍼블리시 파이프라인 완성.
3. **M3. 비전 파이프라인 개발 (Open3D):**
   * 트레이 영역 RANSAC 평면 제거 및 병류 클러스터링/중심점 추출 노드 구현.
4. **M4. MoveIt 2 매니퓰레이션 구현:**
   * UR5e 궤적 생성, 그리퍼 파지/리프트/배치 모션 및 충돌체 회피 구현.
5. **M5. FSM 오케스트레이션 및 예외 처리 통합:**
   * 정상 루프 및 6가지 예외 상황 처리 로직 통합 테스트 및 성능 검증.
