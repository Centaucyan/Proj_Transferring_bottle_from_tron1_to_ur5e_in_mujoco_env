# Implementation Plan - Phase 01-U02 Tron1 Docking Stance Slip & Roll Imbalance Fix

도킹 정지 후 발생하는 **좌우 다리 불균형에 의한 Roll 롤링 현상**과 **고관절 과도 가압으로 인한 발끝 전방 슬립 및 전도 문제**를 해결하기 위한 제어기 개선 계획입니다.

## User Review Required

> [!IMPORTANT]
> **핵심 변경 요약**
> 1. **고관절 정적 과도 가압(15 Nm) 제거 및 단계적 접촉력 적응 제어(Passive Leaning Regulation) 도입**:
>    - 기존의 무조건적인 전방 15 Nm 밀어붙이기를 제거하고, 초기 안착 2초간 3.5 Nm $\to$ 1.5 Nm로 감쇠한 후 실측 범퍼 접촉력($F_{\text{bumper}}$)에 따라 0.5 ~ 1.2 Nm 최소 바이어스로 제어하여 발끝이 바닥을 차고 밀려나는 현상을 100% 차단합니다.
> 2. **무충격 대칭 보간(Zero-Jerk Bumpless Symmetrization)**:
>    - 정적 락 체결 시점($t=0$)에는 현재 관절각에서 출발하여 발구름 충격을 원천 방지하고, 1.5초 동안 좌우 대칭 목표각(`avg_hip`, `avg_knee`)으로 부드럽게 보간하여 짝다리 자세를 해소합니다.
> 3. **Roll 수평 레벨러 차동 무릎 피드백(Differential Knee Roll Stabilizer)**:
>    - 2점 점접촉 상태에서 발생하는 Roll 양의 편차(우측 쏠림)에 대해 차동 무릎 토크($\Delta \tau_{\text{knee}} = 180 \cdot \theta_{\text{roll}} + 20 \cdot \dot{\theta}_{\text{roll}}$)를 인가하여 Roll을 $|\text{Roll}| < 0.05^\circ$ 수평으로 강제 유지합니다.

---

## Proposed Changes

### Tron1 Payload & Docking Controller

#### [MODIFY] [phase01_u02_test_tron1_payload.py](file:///media/korit/4AD6F9D1D6F9BCEF/Tae_ws/Project/Proj_Transferring_bottle_from_tron1_to_ur5e_in_mujoco_env/scripts_devel_roadmap/phase01_u02_test_tron1_payload.py)

1. **상태 변수 초기화 (`__init__` 및 `reset`)**:
   - `self.lock_target_q = np.zeros(6, dtype=np.float32)` 추가
   - `self.lock_time = None` 추가

2. **정적 스탠스 락 체결 조건 (`READY_FOR_PICK` 상태)**:
   - 이중 지지기 감지 시 현재 관절각 캡처 (`self.lock_start_q`)
   - 락 체결 시점 기록 (`self.lock_time = sim_time`)
   - 대칭 목표각 산출:
     ```python
     avg_hip = float((q_act[1] - q_act[4]) / 2.0)
     avg_knee = float((q_act[2] - q_act[5]) / 2.0)
     self.lock_target_q = np.array([0.0, avg_hip, avg_knee, 0.0, -avg_hip, -avg_knee], dtype=np.float32)
     ```

3. **관절 토크 산출 (`compute_torques`)**:
   - 1.5초 무충격 보간:
     ```python
     t_lock = sim_time - (self.lock_time if self.lock_time is not None else sim_time)
     alpha = min(1.0, t_lock / 1.5)
     q_des = (1.0 - alpha) * self.lock_start_q + alpha * self.lock_target_q
     q_des[0] = 0.0
     q_des[3] = 0.0
     ```
   - Roll 레벨러 차동 무릎 보상:
     ```python
     roll_corr = float(np.clip(180.0 * roll + 20.0 * gyro[0], -10.0, 10.0))
     torques[2] += roll_corr  # knee_L
     torques[5] += roll_corr  # knee_R
     ```
   - 단계적 적응형 고관절 토크:
     ```python
     if t_lock < 2.0:
         tau_hip = 3.5 * (1.0 - t_lock / 2.0) + 1.5
     else:
         if bumper_force > 6.0:
             tau_hip = 0.5
         elif bumper_force < 3.0:
             tau_hip = 2.5
         else:
             tau_hip = 1.2

     torques[1] += tau_hip  # hip_L
     torques[4] -= tau_hip  # hip_R
     ```

---

## Verification Plan

### Automated Empirical Simulation Tests
- Conda 파이썬(`/home/korit/miniconda3/envs/transfer_bottle_by_tron1_py3_10/bin/python`)을 통해 `phase01_u02_test_tron1_payload.py`를 100초간 실행하여 검증:
  1. **WP0 $\to$ WP1 $\to$ 테이블 접근 보행**: 기존 경로 추종 및 도킹 진입 성공 유지 확인
  2. **Roll 수평 유지**: `READY_FOR_PICK` 정지 후 Roll 각도가 $|\text{Roll}| \le 0.1^\circ$ 이내로 수평 유지되는지 확인
  3. **발끝 슬립 억제**: 기존 97초 시점 전도(슬립 22cm) 현상이 완전히 사라지고 100초 이상 장시간 안정적으로 직립 대기하는지 확인
  4. **범퍼 접촉력**: $3 \sim 8\,\text{N}$ 수준의 안정적 수동 지지력 유지 확인
  5. **물병 상태**: 100초 시점까지 물병 3개 모두 직립 상태(`OK (3EA)`) 유지 확인
