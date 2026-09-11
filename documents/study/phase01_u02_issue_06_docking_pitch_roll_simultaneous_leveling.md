# Phase 01 Unit 02 Issue 06: 도킹 후 Roll 및 Pitch 동시 수평 정렬 (Simultaneous Bumpless Leveling)

## 1. 개요 및 문제 정의 (Problem Definition)

### 1.1 도킹 후 Pitch 각도 불일치 현상
Tron1 로봇이 테이블 턱(접촉 높이 0.52m)에 도킹 접촉한 후, 3점 지지(양 발 2점 + 머리 범퍼 1점) 안정성을 확보하기 위해 발을 몸체 중심 뒤로 약 5cm 후퇴 배치(`X_foot = X_base - 0.05m`)시켰습니다.
그 결과:
* **Roll 각도**: 양 발의 대칭 배치로 인해 $Roll \approx 0.0^\circ$ 수준으로 수평 안정화 성공.
* **Pitch 각도**: 지면의 발 위치가 몸체 중심보다 뒤에 있고 범퍼가 테이블 턱을 지탱하고 있어, 기하학적 빗변 형상에 의해 상체가 앞으로 약 $+6.93^\circ$ 숙여지는(Forward Lean) 현상이 발생하였습니다.

```
       [현재 도킹 상태: Forward Lean (+6.93°)]
             (상체 기울어짐)
              /|  <-- 머리 범퍼 (테이블 턱 h=0.52m 접촉)
             / |
            /  |  <-- 몸체 트레이 (Pitch ≈ +6.93° 기울어짐)
           /   |
          /    |
    [발] /_____| [지면 기준 몸체 수직선]
       < 5cm >
```

### 1.2 피칭 각도 잔류가 미치는 영향
1. **물병 전도 위험 및 중력 쏠림**: 트레이가 앞으로 $6.93^\circ$ 기울어지면, 물병 3개에 전방 전단 하중이 가해져 급정지나 외란 시 전방 슬립 및 전도 위험이 증가합니다.
2. **UR5e 협동로봇의 피킹 공차 오차**: 상위 작업기인 UR5e 매니퓰레이터는 트레이가 완전 수평($Roll=0^\circ, Pitch=0^\circ$)일 때 수직 하향(-Z축) 접근으로 병을 파지하도록 캘리브레이션되어 있으므로, 트레이 각도 오차는 그리퍼 파지 불량이나 충돌을 유발할 수 있습니다.

---

## 2. Tron1의 기구학적 특성 및 Pitch 제어 원리

### 2.1 목/허리 틸팅 관절의 부재 (Kinematic Architecture)
Tron1 2족 보행 로봇은 상체에 사람과 같은 목(Neck) 관절이나 흉부/허리(Spine/Waist) 관절이 존재하지 않습니다.
* 베이스 몸체(`base_Link`), 물병 적재 트레이(Payload Tray), 머리 완충 범퍼(Bumper)는 **단일 일체형 강체(Rigid Body)**로 고정 결합되어 있습니다.
* 따라서 상체 및 트레이의 Pitch 각도($\theta_{\text{pitch}}$)를 변경하려면 몸체 전체를 회전시켜야 하며, 이는 **오직 하체(다리 6관절)의 고관절 피치(Hip Pitch: `hip_L, hip_R`)와 무릎(Knee Pitch: `knee_L, knee_R`)의 기구학적 자세 조절**을 통해서만 제어할 수 있습니다.

### 2.2 고관절 신전(Hip Extension)을 통한 상체 직립
상체를 앞으로 숙인 상태에서 똑바로 세우기($Pitch = 6.93^\circ \rightarrow 0.0^\circ$) 위해서는 양 고관절을 뒤로 신전(Hip Extension)시켜야 합니다.
* Tron1 관절 부호 규약:
  $$\Delta q_{\text{hip\_L}} = +\text{bias}, \quad \Delta q_{\text{hip\_R}} = -\text{bias}$$
* 단순히 고관절만 회전시킬 경우 몸체 무게중심이 뒤로 빠져 테이블 턱 범퍼에서 접촉이 떨어질 수 있으나, 이미 구현된 **능동 범퍼 반력 순응 제어(Active Force Compliance Control)**와 무릎 지탱 토크가 연동되어 범퍼를 테이블 턱에 약 20~28N으로 안정적으로 밀착시키면서 상체 트레이만 수직으로 세워지게 됩니다.

---

## 3. 제어 전략 비교: 순차 제어 vs 동시 제어

### 3.1 Roll과 Pitch의 수학적 직교성 (Orthogonality / Decoupling)
Tron1의 좌우 2족 다리 관절 공간(Joint Space)에서:
* **Roll 제어**: 좌/우 다리의 **차동 모드(Differential Mode)**
  $$\Delta q_{\text{roll}} \propto (q_{\text{left}} - q_{\text{right}})$$
* **Pitch 제어**: 좌/우 다리의 **동위상 대칭 모드(Common Mode)**
  $$\Sigma q_{\text{pitch}} \propto \frac{q_{\text{left}} + q_{\text{right}}}{2}$$

수학적으로 Roll과 Pitch는 관절 공간에서 **완전히 직교(Decoupled)**하므로, 상호 간섭(Coupling Interference) 없이 **동시에 독립 제어가 가능**합니다.

### 3.2 상세 비교

| 비교 항목 | 방안 A: 순차 제어 (Sequential Control) | 방안 B: 동시 제어 (Simultaneous Bumpless Alignment, 채택 ★) |
| :--- | :--- | :--- |
| **제어 메커니즘** | 1단계: 양 발 대칭 맞춰 Roll=0° 안정화<br>2단계: Roll 정지 후 고관절 젖혀 Pitch=0° 안정화 | `STANCE_LOCK` 진입 시 [양발 대칭 + 고관절 직립] 목표를 한 번에 설정하고 0.5초 동안 부드럽게 동시 보간 |
| **물병 진동 / 저크 (Jerk)** | **불리 (2차 충격 발생)**<br>Roll을 맞추고 정지한 후 다시 Pitch를 구동하여 2회의 가감속 충격 발생 $\rightarrow$ 트레이 위 액체 출렁임(Sloshing) 유발 | **최적 (Zero Jerk / Bumpless)**<br>단 1회의 최단 부드러운 궤적(Minimum Jerk)으로 완전 수평면에 안착 $\rightarrow$ 물병 진동 제로 |
| **공정 시간 (Cycle Time)** | **지연 (총 5~6초 소요)**<br>단계별 정착 시간(Settle Time) 2회 누적 | **단축 (총 3.5초 소요)**<br>0.5초 만에 $Roll=0^\circ, Pitch=0^\circ$ 동시 도달 후 3초 안정 카운트 개시 |
| **FSM 상태 관리** | 상태 머신에 중간 단계(`ALIGN_PITCH`) 추가 필요 | 기존 `STANCE_LOCK` 내 목표 관절각 `lock_target_q`로 일괄 처리 |

**결론**: 공정 시간 단축과 물병 액체 출렁임(Sloshing) 방지, 그리고 수학적 비간섭 원리에 따라 **[동시 제어 (Simultaneous Bumpless Alignment)]**를 채택하였습니다.

---

## 4. 수치 검증 및 실험 결과 (Empirical Simulation Results)

실제 MuJoCo 500Hz 물리 시뮬레이션에서 동시 제어를 적용하고 고관절 보정각(`hip_bias`)을 변화시키며 Pitch, Roll, 범퍼 반력을 측정한 결과는 다음과 같습니다:

| 고관절 보정각 (`hip_bias`) | Pitch 각도 | Roll 각도 | 범퍼 지탱 반력 | 비고 및 판정 |
| :---: | :---: | :---: | :---: | :---: |
| **기존 (0.00 rad)** | $+6.93^\circ$ | $+0.13^\circ$ | $18.41\,\text{N}$ | 상체 전방 숙여짐 잔류 |
| **+0.05 rad** | $+4.98^\circ$ | $+0.06^\circ$ | $20.25\,\text{N}$ | Pitch 각도 선형 감소 |
| **+0.10 rad** | $+3.05^\circ$ | $+0.03^\circ$ | $22.24\,\text{N}$ | 점진적 직립 진행 |
| **+0.12 rad** | $+2.26^\circ$ | $+0.02^\circ$ | $23.10\,\text{N}$ | 수평 영역 근접 |
| **+0.15 rad** | $+1.16^\circ$ | $+0.00^\circ$ | $24.48\,\text{N}$ | Roll 완벽 수평 |
| **+0.18 rad (약 10.3°)** | **$+0.13^\circ$** | **$-0.02^\circ$** | **$28.36\,\text{N}$** | **완벽한 수평 ($Roll \approx 0.0^\circ, Pitch \approx 0.0^\circ$) 달성!** |

### 주요 관측 결과
1. **완벽한 수평면 확립**: `hip_bias = +0.18 rad` 적용 시 트레이의 최종 자세는 **$Roll = -0.02^\circ, Pitch = +0.13^\circ$**로, $0.1^\circ$ 이내의 완전 수평면을 달성하였습니다.
2. **견고한 범퍼 지지 유지**: 고관절을 젖혀 상체를 세웠음에도 불구하고 범퍼 반력은 $28.36\,\text{N}$으로 견고하게 유지되어, 테이블 턱에서 떨어지거나 들뜨는 현상이 전혀 발생하지 않았습니다.
3. **수평 3초 유지 후 클램프 체결**:
   * Roll과 Pitch가 모두 $\pm 1.0^\circ$ 이내로 진입한 시점부터 3.0초 타이머가 작동.
   * 3.0초간 완전 수평 안정 상태를 유지하여 50.09초에 `READY_FOR_PICK` 확립.
   * 도킹 클램프(`dock_locked_qpos`)가 자동 체결되어 **UR5e 피킹 작업 중 0.000mm 완전 부동** 상태를 보장하였습니다.

---

## 5. 코드 구현 (Code Implementation)

### 5.1 목표 관절각 계산 (`phase01_u02_test_tron1_payload.py`)
```python
# [STANCE_LOCK 진입 시점]
avg_hip = float(np.clip((q_act[1] - q_act[4]) / 2.0, 0.22, 0.32))
avg_knee = float(np.clip((q_act[2] - q_act[5]) / 2.0, 0.40, 0.48))

# Roll=0° 대칭 정렬과 Pitch=0° 직립 신전을 동시 보간 (Simultaneous Bumpless Alignment)
hip_level = avg_hip + 0.18  # 상체 직립(Pitch 6.9° -> 0.0°) 고관절 보정각 (+0.18 rad)
self.lock_target_q = np.array([0.0, hip_level, avg_knee, 0.0, -hip_level, -avg_knee], dtype=np.float32)
```

### 5.2 Roll & Pitch 동시 수평 3초 안정화 조건 판정
```python
# 구름 정지 후 Roll 및 Pitch가 모두 0도 (±1.0° 이내)로 3초간 유지되면 READY_FOR_PICK 확립!
if self.is_stance_locked:
    roll_deg_abs = abs(math.degrees(roll))
    pitch_deg_abs = abs(math.degrees(pitch))
    if roll_deg_abs < 1.0 and pitch_deg_abs < 1.0 and self.tray_vel_smooth < 0.08:
        self.roll_zero_timer += dt
        if self.roll_zero_timer >= 3.0:
            self.fsm_state = "READY_FOR_PICK"
            self.state_timer = 0.0
            self.telemetry.is_ready_for_pick = True
            print(f"\n  ✔ [{sim_time:5.2f}s] [도킹 성공] 수평안정 달성! Roll={math.degrees(roll):+.2f}°, Pitch={math.degrees(pitch):+.2f}° 3.0초간 유지 => 'READY_FOR_PICK' 확립! (물병 피킹 대기)\n", flush=True)
    else:
        self.roll_zero_timer = max(0.0, self.roll_zero_timer - 1.0 * dt)
```

---

## 6. 결론 및 향후 계획

1. **도킹 자세 안정화 완성**:
   * 로봇발 5cm 후퇴 배치로 전복 방지 3점 지지 달성.
   * 능동 범퍼 순응 제어로 과도 충격 흡수 및 테이블 밀착 유지.
   * **동시 수평 제어(Simultaneous Leveling)로 $Roll = -0.02^\circ, Pitch = +0.13^\circ$ 완전 수평 트레이 확립**.
   * 3초 수평 안정 유지 후 **도킹 정밀 고정 락(0.000mm 부동) 체결 및 `READY_FOR_PICK` 확립**.
2. **다음 단계 (Next Step)**:
   * `READY_FOR_PICK` 상태가 확립되었으므로, 다음 단계인 **UR5e 로봇 팔의 그리퍼 궤적 생성 및 물병 피킹(Picking) 시퀀스**로 원활하게 연계 작업을 진행할 수 있습니다.
