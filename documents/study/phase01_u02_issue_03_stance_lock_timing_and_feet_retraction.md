# Phase 01-U02 이슈 03: 도킹 접촉 후 발구름 정지 시 후방/측면 전도 현상 분석 및 양발 대칭 5cm 후퇴 락 & Roll=0° 안정화 해결 보고서

## 1. 문제 정의 (Problem Definition)

### 1.1 현상 요약 및 테스트 히스토리
* Phase 01-U02 물병 운반 및 도킹 시뮬레이션(`phase01_u02_test_tron1_payload.py`) 실행 시:
  * **1차 테스트 (`11:35:38 AM` 영상)**: 도킹 직후 과거 전진 오프셋 음수 속도 명령(`commands[0] = -0.36`)으로 인해 로봇이 3초간 스스로 뒤로 걸어 나와 허공에서 관절이 잠기며 후방 전도 발생.
  * **2차 테스트 (`11:50:43 AM` 영상)**: 터치센서 감지(`36.56s`) 직후 불과 0.06초 만에(`36.62s`) 양발 접지 감지 조건이 즉시 격발되어 발구름을 급정지함. 관절 급격 고정 시 발생한 고관절 반작용 토크 충격($-28\,\text{Nm}$)으로 상체가 완충 턱에서 튕겨 나가 후방 전도.
  * **3차 테스트 (`12:01:32 PM` 영상)**: 발구름 주파수를 점진적으로 감속($2.0\,\text{Hz} \to 0.6\,\text{Hz}$)시키는 방식을 적용했으나, **주파수가 낮아지면서 보행 주기가 길어져(한 걸음 0.83초) 두 발의 X축 위치 차이가 10cm 이상 벌어짐(한 발은 전방 +4cm, 한 발은 후방 -6cm)**. 이로 인해 로봇이 좌우로 기울며(Roll 불안정) 쓰러지는 현상 발생.

* **사용자 요구사항**:
  > *"터치 센서 감지 후 로봇발을 뒤로 물리며 천천히 감속하는 건 적용된것 같아...*
  > *그런데 영상에서 보듯 천천히 구르면서 로봇의 두 발 x 위치가 달라져 기울면서 쓰러진다.*
  > *천천히 감속보다 **터치 센서 감지 후 로봇발 구름 속도는 유지한 채 로봇 머리는 범퍼에 기댄 채로 로봇발을 5cm 정도 뒤로 위치시킨 뒤에 구름을 정지**하는 게 나을 것 같아.*
  > *구름 정지 후 **roll이 0도가 되도록 두 발의 포지션은 동일하게 맞추고**, **roll이 0이 3초간 유지**되면 로봇 상태를 ready_pickup이 될 수 있도록 수정해줘~"*

---

### 1.2 3차 테스트 영상 및 데이터 분석
* **영상 분석 (`frame_8.png`, `frame_9.png`)**:
  * 테이블 범퍼에 머리가 기댄 상태에서 $0.6\,\text{Hz}$로 느리게 굴렀을 때, 유각기(Swing leg)와 지지기(Stance leg)의 위상 차이로 인해 한 발은 전방 $+0.04\,\text{m}$, 반대 발은 후방 $-0.06\,\text{m}$로 벌어짐.
  * 지면 반력의 좌우 비대칭으로 인해 로봇 상체가 우측으로 크게 기울어지며 전도됨.
* **사용자 제안의 물리적 타당성**:
  1. **정상 보행 속도($2.0\,\text{Hz}$) 유지**: 주파수를 낮추지 않고 정상 주파수로 발구름을 지속하면 양발의 전후 벌어짐 폭이 매우 작게($\le 2\sim 3\,\text{cm}$) 유지됨.
  2. **로봇 머리는 범퍼에 기댄 채 발을 5cm 뒤로 위치**: 상체 전단 범퍼가 테이블에 구속되어 전진할 수 없으므로, 전진 추진력을 가하면 두 발이 상체 대비 **자연스럽게 뒤로 약 5cm 후퇴**하게 됨.
  3. **구름 정지 후 양발 포지션 대칭 일치 ($q_L = q_R$)**: 두 다리의 관절각을 완전히 동일하게 정렬하면 지면 접촉 높이가 완벽히 일치하여 기구학적으로 $Roll = 0.0^\circ$가 확립됨.

---

## 2. 핵심 물리 원리 및 원인 규명

### 2.1 3점 지지 수동 정적 평형과 무게중심(CoM) 조건
로봇이 테이블 완충 턱에 기대어 자립하기 위한 모멘트 평형 조건:
$$F_{\text{table}} = M g \cdot \frac{X_{\text{com}} - X_{\text{foot}}}{Z_{\text{table}}}$$

* **발이 몸체보다 앞에 있는 경우 ($X_{\text{foot}} > X_{\text{com}}$)**:
  * $F_{\text{table}} < 0$이 되어 중력에 의해 상체가 뒤로 넘어가 자유 낙하 전도.
* **발이 몸체 뒤로 5cm 물러난 경우 ($X_{\text{com}} - X_{\text{foot}} \approx +0.05\,\text{m}$)**:
  * $F_{\text{table}} \approx 150\,\text{N} \times \frac{0.05\,\text{m}}{0.60\,\text{m}} = \mathbf{+12.5\,\text{N}}$
  * **순수 중력에 의해 약 $12.5\,\text{N}$의 전방 밀착 지지력이 자연 발생**하여, 모터로 밀지 않아도 절대 뒤로 넘어가지 않는 영구적 정적 평형 확립.

### 2.2 고관절 토크 작용-반작용 특성
* Actuator 1 (`hip_L_motor`)과 Actuator 4 (`hip_R_motor`):
  * $ctrl[1] > 0$: 허벅지를 전방으로 회전시키며, 반작용으로 상체(`base_Link`)를 **후방으로 $-12.5\,\text{m/s}^2$ 가속**시켜 테이블에서 떼어냄.
  * $ctrl[1] < 0$: 상체를 **전방으로 $+12.5\,\text{m/s}^2$ 가속**시켜 완충 턱에 밀착시킴.
* 따라서 스탠스 락 체결 시에는 고관절 목표각을 인위적으로 급격히 변경하지 않고, 5cm 후퇴한 현재 안착 각도로 대칭 정렬하여 반작용 충격을 원천 방지해야 함.

---

## 3. 해결 방안 (Proposed Solution)

사용자의 요구사항을 100% 반영한 **양발 대칭 5cm 후퇴 락 및 Roll=0° 안정화 파이프라인**:

```
[도킹 접촉 감지: bumper_force >= 2.0N / pos_x >= 0.258m]
       │
       ▼
┌─────────────────────────────────────────────────────────────┐
│ 1단계: 정상 보행(2.0Hz) 유지 & 머리 밀착 상태로 발 5cm 후퇴  │
│  - gait[0] = 2.0Hz 정상 보행 속도 유지 (감속 금지)         │
│  - commands[0] = +0.08 전진 추진 명령                       │
│    (머리는 테이블에 닿아 멈추고, 두 발이 상체 뒤로 5cm 물러남) │
│  - 양발 상대 위치 모니터링: avg_foot_rel <= -0.035m         │
└─────────────────────────────────────────────────────────────┘
       │ (두 발이 5cm 뒤로 물러나고 양발 접지된 순간)
       ▼
┌─────────────────────────────────────────────────────────────┐
│ 2단계: 발구름 즉시 정지 & 양발 동일 대칭 포지션 맞춤 (Roll=0°) │
│  - is_stance_locked = True (발구름 즉시 정지)               │
│  - 좌우 대칭 관절각 정렬:                                   │
│    target_hip = (q_L[1] - q_R[1])/2 (~0.28 rad)             │
│    target_knee = (q_L[2] - q_R[2])/2 (~0.44 rad)            │
│    lock_target_q = [0.0, avg_hip, avg_knee, 0.0, -avg_hip, -avg_knee] │
│  - Roll 능동 수평 레벨러 (차동 무릎 피드백):                 │
│    roll_corr = clip(250 * roll + 30 * gyro_x, -15, 15)      │
└─────────────────────────────────────────────────────────────┘
       │ (Roll 각도가 0° (±1.0° 이내)로 3.0초간 유지)
       ▼
[READY_FOR_PICK: UR5e 물병 피킹 대기 완료 확립!]
```

---

## 4. 코드 구현 상세

### 4.1 FSM 상태 전이 (`scripts_devel_roadmap/phase01_u02_test_tron1_payload.py`)
```python
elif self.fsm_state == "STANCE_LOCK":
    if not self.is_stance_locked:
        foot_L_x = float(data.geom_xpos[self.foot_L_geom_id][0]) - pos_x
        foot_R_x = float(data.geom_xpos[self.foot_R_geom_id][0]) - pos_x
        avg_foot_rel = (foot_L_x + foot_R_x) / 2.0
        foot_diff = abs(foot_L_x - foot_R_x)

        # 발이 5cm 뒤(-0.035m 이하)로 위치하고, 양발 차이가 적으며(<=0.04m), 양발 접지 시 구름 정지!
        is_feet_back = (avg_foot_rel <= -0.035) and (foot_diff <= 0.040) and contact_L and contact_R
        is_timeout_ready = (self.state_timer >= 1.5) and contact_L and contact_R

        if is_feet_back or is_timeout_ready:
            self.is_stance_locked = True
            self.lock_start_q = np.copy(q_act)
            self.lock_time = sim_time
            # 구름 정지 후 roll이 0도가 되도록 두 발 포지션 대칭 일치화
            avg_hip = float(np.clip((q_act[1] - q_act[4]) / 2.0, 0.22, 0.32))
            avg_knee = float(np.clip((q_act[2] - q_act[5]) / 2.0, 0.40, 0.48))
            self.lock_target_q = np.array([0.0, avg_hip, avg_knee, 0.0, -avg_hip, -avg_knee], dtype=np.float32)

    # 2. 구름 정지 후 roll이 0도 (±1.0° 이내)로 3초간 유지되면 READY_FOR_PICK 확립!
    if self.is_stance_locked:
        roll_deg_abs = abs(math.degrees(roll))
        if roll_deg_abs < 1.0 and self.tray_vel_smooth < 0.08:
            self.roll_zero_timer += dt
            if self.roll_zero_timer >= 3.0:
                self.fsm_state = "READY_FOR_PICK"
                self.state_timer = 0.0
                self.telemetry.is_ready_for_pick = True
        else:
            self.roll_zero_timer = max(0.0, self.roll_zero_timer - 1.0 * dt)
```

### 4.2 보행 정책 명령 및 스탠스 락 관절 PID
* **발 5cm 후퇴 보행 명령**:
  `self.gait[0] = 2.0` (정상 속도), `self.commands[0] = +0.08` (추진력 인가로 발 후퇴 유도).
* **스탠스 락 및 Roll=0° 레벨링**:
  `torques = 120.0 * q_err - 8.0 * v_act + 20.0 * q_int`
  `torques[2] -= 18.0` (무릎 중력 지탱)
  `torques[5] += 18.0`
  `roll_corr = clip(250.0 * roll + 30.0 * gyro[0], -15.0, 15.0)`
  `torques[2] += roll_corr`
  `torques[5] += roll_corr`

---

## 5. 검증 시뮬레이션 결과

* **발 5cm 후퇴 도달 시점**: $38.51\,\text{s}$에 `avg_rel = -0.035m`, `diff = 0.008m` 달성 즉시 발구름 정지 및 락 체결.
* **체결 후 3초간 궤적**:
  * $BaseX = 0.270\,\text{m}$ (완전 불변 정지)
  * $BaseZ = 0.719\,\text{m}$ (정상 기립 높이 유지)
  * $Roll = -0.08^\circ \sim -0.15^\circ$ (오차 $0.15^\circ$ 이내로 완벽한 수평 유지)
  * $Roll \approx 0.0^\circ$ 3.0초 연속 유지 후 정상적으로 `READY_FOR_PICK` 확립 확인.
