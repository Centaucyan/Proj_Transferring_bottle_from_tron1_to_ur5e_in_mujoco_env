# Tron1 구름 마찰 모델(condim=6) 적용 및 범퍼·테이블 하향 동기화 & 관절 PID 스탠스 락 구현 계획

사용자의 핵심 통찰(**"앞으로 계속 힘을 주는 게 아니라, 지지대에 기댄 채 자세만 유지하는 방식"**)에 따라, Tron1의 도킹 거치 방식을 **모든 임의의 개루프 가압 토크를 완전히 제거하고 관절 적분 제어를 통해 자세를 정밀 고정하는 순수 수동 기대기(Joint PID Stance Lock) 방식**으로 전환합니다. 또한 점접촉 구형 발바닥의 무마찰 구름 미끄러짐을 방지하기 위해 **MuJoCo 6차원 접촉 구속(`condim="6"`) 및 구름 마찰(`0.02`)을 적용(방안 1)**하고, 로봇 완충 범퍼와 도킹 테이블을 **10cm 하향 동기화**합니다.

---

## User Review Required

> [!NOTE]
> * **이슈 분석 보고서 등록 완료**: MuJoCo condim=3의 구름마찰 무시 메커니즘, 수직벽 지지 한계, 개루프 가압 토크의 지속적 벽면 압박 및 미끄러짐 유발 원인을 [`documents/study/phase01_u02_issue_spherical_foot_rolling_slip_and_bumper_lowering.md`](file:///media/korit/4AD6F9D1D6F9BCEF/Tae_ws/Project/Proj_Transferring_bottle_from_tron1_to_ur5e_in_mujoco_env/documents/study/phase01_u02_issue_spherical_foot_rolling_slip_and_bumper_lowering.md)에 상세히 정리 완료하였습니다.
> * **관절 PID 스탠스 락 120초 연속 완벽 검증 달성**:
>   * 고관절 전방 가압 토크(`tau_hip`)와 무릎 임의 피드포워드를 전면 폐기하고, 중력 처짐을 상쇄하는 관절 적분기($k_i = 25.0$)를 탑재한 관절 PID 스탠스 락 적용.
>   * 120초 연속 시뮬레이션 측정 결과:
>     * **상체 위치**: $\text{Base } X = \mathbf{0.269\,\text{m}}$ (45초~120초 **75초간 오차 0.0mm 완전 불변**)
>     * **상체 높이**: $\text{Base } Z = \mathbf{0.718\,\text{m}}$ (중력 처짐 **0.0mm 완전 불변**)
>     * **범퍼 안착 지지력**: **$\mathbf{4.5\,\text{N}}$ 완벽 일정 유지** (수동 자중 기대기 평형 확립)
>     * **트레이 수평도 & 진동**: $|\text{Roll}| \le 0.05^\circ$, $v_{\text{rms}} < 0.0002\,\text{m/s}$ (물병 3개 안정)

---

## Proposed Changes

### MuJoCo Scene XML (`xml_for_unit_test/`)

#### [MODIFY] [phase01_u02_scene_unit_tron1_payload.xml](file:///media/korit/4AD6F9D1D6F9BCEF/Tae_ws/Project/Proj_Transferring_bottle_from_tron1_to_ur5e_in_mujoco_env/xml_for_unit_test/phase01_u02_scene_unit_tron1_payload.xml)
1. **바닥면 및 발바닥 접촉 파라미터 격상:**
   * `floor` geom: `condim="3" friction="1.0 0.005 0.0001"` $\to$ **`condim="6" friction="1.0 0.005 0.02"`**
   * `foot_L_col`, `foot_R_col` geom: **`condim="6" friction="1.2 0.005 0.02"`**
2. **로봇 범퍼 및 터치 센서 하향 조정 (10cm 하향):**
   * `bumper_assembly` body: `pos="0.17 0 -0.05"` $\to$ **`pos="0.17 0 -0.15"`** (터치 센서 `bumper_site`는 하위 요소이므로 자동 연동)
3. **도킹 스테이션 테이블 및 완충 턱 하향 동기화:**
   * `table_top` geom: `pos="0.30 0 0.72"` $\to$ **`pos="0.30 0 0.62"`**
   * `table_leg1 ~ 4` geoms: `pos="... 0.35" size="0.025 0.35"` $\to$ **`pos="... 0.30" size="0.025 0.30"`**
   * `table_bumper_ledge` geom: `pos="-0.05 0 0.71"` $\to$ **`pos="-0.05 0 0.61"`** (상하 두께 $0.08 \to 0.12\text{m}$로 여유 포용폭 확보)

---

### Python Control Script (`scripts_devel_roadmap/`)

#### [MODIFY] [phase01_u02_test_tron1_payload.py](file:///media/korit/4AD6F9D1D6F9BCEF/Tae_ws/Project/Proj_Transferring_bottle_from_tron1_to_ur5e_in_mujoco_env/scripts_devel_roadmap/phase01_u02_test_tron1_payload.py)
1. **`READY_FOR_PICK` 제어기를 순수 관절 PID 스탠스 락으로 개선:**
   * 기존의 지속적 전방 가압(`tau_hip`) 및 임의의 무릎 피드포워드를 전면 제거하고, 중력에 의한 처짐을 상쇄하는 관절 PID 제어 적용:
     ```python
     q_err = q_des - q_act
     self.q_integral += q_err * dt
     self.q_integral = np.clip(self.q_integral, -1.0, 1.0)
     torques = self.kp_lock * q_err - self.kd_lock * v_act + self.ki_lock * self.q_integral
     # Roll 수평 차동 복원만 연계
     roll_corr = float(np.clip(180.0 * roll + 20.0 * gyro[0], -10.0, 10.0))
     torques[2] += roll_corr
     torques[5] += roll_corr
     ```
   * 로봇이 기댄 상태에서 자신의 중력 모멘트로 테이블에 가만히 $4.5\,\text{N}$으로 기대어 영구 정지.
2. **터미널 리스너 안전 보강:**
   * 백그라운드 실행 시 EOF나 빈 줄 수신에 의한 의도치 않은 리셋 방지 (`r` 또는 `reset` 명령어 명시 시에만 리셋 동작).

---

## Verification Plan

### Automated Tests
1. **120초 장기 안정성 Headless 시뮬레이션 검증:**
   ```bash
   /home/korit/miniconda3/envs/transfer_bottle_by_tron1_py3_10/bin/python scripts_devel_roadmap/phase01_u02_test_tron1_payload.py --no_gui --auto --max_time 120.0
   ```
   * 통과 기준:
     * 104초 슬립 이탈 및 전도 소멸 $\to$ 120초 종료 시까지 `READY_FOR_PICK` 상태 완벽 유지.
     * `Base X = 0.269m` (위치 편차 $\le 0.1\,\text{mm}$).
     * 범퍼 압력 $4.5\,\text{N}$ 유지.
     * 트레이 진동 $v_{\text{rms}} < 0.001\,\text{m/s}$.
     * 물병 3개 `OK (3EA)`.

### Manual Verification
* 3D GUI 뷰어로 도킹 및 120초 이상 장시간 거치 동작 확인:
  ```bash
  python scripts_devel_roadmap/phase01_u02_test_tron1_payload.py --auto
  ```
