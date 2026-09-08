#!/usr/bin/env python3
"""
scripts_devel_roadmap/phase01_u01_test_tron1_walking.py
Phase 01-U01 Step 1: Tron1 Spawn Pose Holding & Joint PD Stabilization
- Loads initial spawn posture from XML 'stand' keyframe (or fallback)
- Firmly maintains the spawn posture via Joint PD control (locomotion disabled for step-by-step rebuilding)
- 1.0x Real-time Physics Speed Synchronization
- Interactive Viewer Auto-Reset Support (Instant sync on Backspace/Reset)
- Telemetry monitoring: Base height Z, gyro rates, and joint tracking errors
"""

import os
import sys
import argparse
import math
import time
import threading
from dataclasses import dataclass
import numpy as np
import mujoco
import mujoco.viewer

@dataclass
class RobotState:
    """
    Tron1 로봇 상태 통합 관리 데이터클래스
    - FSM 제어 상태: 'landing' -> 'ready' -> 'standing_up' -> 'stepping' (실패 시 'falling')
    - 상체 위치 및 IMU 센서 데이터
    - 수평 유지 시간 및 전도 추적
    """
    state: str = "landing"         # FSM 상태 ('landing' -> 'ready' -> 'standing_up' -> 'stepping', 실패 시 'falling')
    sim_time: float = 0.0          # 시뮬레이션 경과 시간 (초)
    pos_x: float = 0.0             # 상체 X 위치 (m)
    pos_z: float = 0.0             # 상체 Z 높이 (m)
    pitch: float = 0.0             # IMU Pitch 각도 (rad)
    pitch_deg: float = 0.0         # IMU Pitch 각도 (deg)
    pitch_rate: float = 0.0        # Pitch 회전 각속도 (rad/s)
    gyro_norm: float = 0.0         # IMU 3축 회전 각속도 크기 (rad/s)
    touch_L: bool = False          # 왼발 지면 접촉 여부 (MuJoCo 물리 충돌 검출)
    touch_R: bool = False          # 오른발 지면 접촉 여부 (MuJoCo 물리 충돌 검출)
    touch_L_force: float = 0.0    # 왼발 지면 접촉 반력 (N)
    touch_R_force: float = 0.0    # 오른발 지면 접촉 반력 (N)
    stable_duration: float = 0.0   # IMU 값 불변(안정화) 연속 유지 시간 (초)
    tilt_duration: float = 0.0     # 수평 벗어남 누적 지속 시간 (초)
    max_joint_error: float = 0.0   # 6개 관절 중 최대 추종 오차 (rad)
    is_fallen: bool = False        # 전도 여부 플래그

class Colors:
    GREEN = '\033[92m'
    RED = '\033[91m'
    YELLOW = '\033[93m'
    CYAN = '\033[96m'
    BOLD = '\033[1m'
    RESET = '\033[0m'

def parse_args():
    parser = argparse.ArgumentParser(description="Phase 01-U01: Tron1 Spawn, Stand-Up & In-place Stepping")
    parser.add_argument("--headless", action="store_true", help="Run in headless text-only mode (no GUI window)")
    parser.add_argument("--xml", type=str, default="unit_test_models/phase01_u01_scene_unit_tron1.xml",
                        help="Path to Tron1 unit scene XML")
    parser.add_argument("--max_time", type=float, default=20.0, help="Maximum simulation time limit in seconds (headless mode)")
    return parser.parse_args()

class Tron1PoseHoldController:
    """
    Tron1 직립 착지 및 제자리 발구름(In-place Stepping) 동적 균형 제어기
    - landing: 직립 스폰 후 발끝 지면 착지 및 IMU 안정화 확인 (1.5초)
    - ready: 발구름 준비 대기 (1.5초)
    - stepping: Raibert 피치 적응형 제자리 발구름(In-place Stepping)으로 영구 직립 균형 유지
    """
    def __init__(self, model):
        self.model = model

        self.actuator_names = [
            "abad_L_motor", "hip_L_motor", "knee_L_motor",
            "abad_R_motor", "hip_R_motor", "knee_R_motor"
        ]
        
        self.act_ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name) for name in self.actuator_names]
        self.base_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_Link")
        self.foot_L_geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "foot_L_col")
        self.foot_R_geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "foot_R_col")

        # 1. 스폰 직립 자세 (XML의 'stand' 키프레임에서 자동 로드)
        stand_key_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "stand")
        if stand_key_id != -1:
            self.q_stand = model.key_qpos[stand_key_id][7:13].copy()
        else:
            self.q_stand = np.array([0.0, 0.10, 0.09, 0.0, -0.10, -0.09])

        # 현재 추종 목표 각도 (직립 자세로 시작)
        self.q_target = self.q_stand.copy()
        self.target_hip_L = self.q_stand[1]
        self.target_hip_R = self.q_stand[4]

        # 관절 PD 게인 (직립 지지력 및 무릎 굴곡 댐핑 강화)
        self.kp = np.array([300.0, 320.0, 380.0,  300.0, 320.0, 380.0])
        self.kd = np.array([18.0, 20.0, 24.0,     18.0, 20.0, 24.0])

        # IMU 상체 수평(Pitch) 능동 제어 게인 (고관절 댐핑 보조)
        self.kp_pitch = 25.0
        self.kd_pitch = 2.5
        self.pitch_integral = 0.0

        # Raibert 동적 발 착지(Foot Placement) 적응 게인
        self.k_raibert = 0.35        # 피치 기울기에 따른 유각 발끝 착지 오프셋
        self.k_raibert_rate = 0.05   # 피치 회전 각속도 댐핑 오프셋

        # FSM 타이머 및 전이 파라미터
        self.pitch_tolerance = math.radians(4.0)  # 수평 판정 허용 각도 (±4.0°)
        self.tilt_max_time = 3.0                 # 수평 불량 지속 허용 한계 시간 (초)
        self.landing_duration = 0.8              # landing 지면 안착 기준 시간 (0.8초)
        self.ready_duration = 0.5                # ready 직립 안정화 대기 시간 (0.5초)
        self.landing_timer = 0.0
        self.ready_timer = 0.0
        self.stepping_timer = 0.0                # 제자리 발구름 지속 시간

        # 제자리 발구름(In-place Stepping) 파라미터
        self.gait_period = 0.28                  # 보행 주기 (초, 약 3.6Hz 고주파 발구름)
        self.step_height = 0.035                 # 발 들어올리는 무릎 굽힘 높이 (rad, 약 1.5cm 지면 이격)
        self.roll_sway = 0.010                   # 발 디딤 시 골반 좌우 무게중심 이동 각도 (rad)

        self.robot_state = RobotState(state="landing")
        self.reset()

    def set_state(self, new_state, reason=""):
        """상태 전이 처리 및 터미널 강조 출력"""
        old_state = self.robot_state.state
        if old_state == new_state:
            return
        self.robot_state.state = new_state
        reason_str = f" ({reason})" if reason else ""
        if new_state == "falling":
            print(f"\n  {Colors.BOLD}{Colors.RED}✗ [{self.robot_state.sim_time:5.2f}s] [상태 전이] '{old_state}' ➔ '{new_state}'{reason_str}{Colors.RESET}\n", flush=True)
        else:
            print(f"\n  {Colors.BOLD}{Colors.CYAN}★ [{self.robot_state.sim_time:5.2f}s] [상태 전이] '{old_state}' ➔ '{new_state}'{reason_str}{Colors.RESET}\n", flush=True)

    def reset(self):
        self.robot_state = RobotState(state="landing")
        self.landing_timer = 0.0
        self.ready_timer = 0.0
        self.stepping_timer = 0.0
        self.q_target = self.q_stand.copy()
        self.target_hip_L = self.q_stand[1]
        self.target_hip_R = self.q_stand[4]
        self.pitch_integral = 0.0
        self.current_pitch = 0.0
        self.tilt_duration = 0.0
        self.log_time = []
        self.log_z = []
        self.log_pitch = []
        self.log_gyro = []
        self.log_max_error = []
        self.log_state = []
        self.log_fall = False

    def compute_torques(self, data):
        sim_time = data.time
        pos_x = data.xpos[self.base_body_id][0]
        pos_z = data.xpos[self.base_body_id][2]

        q_act = data.qpos[7:13]
        v_act = data.qvel[6:12]
        gyro = data.sensor("imu_gyro").data.copy()

        # =========================================================================
        # 1. IMU 쿼터니언에서 상체 앞뒤 기울기(Pitch) 및 회전 속도 추출
        # =========================================================================
        quat = data.sensor("imu_quat").data
        w, x, y, z = quat
        sinp = 2.0 * (w * y - z * x)
        pitch = math.asin(np.clip(sinp, -1.0, 1.0))  # 뒤로 들리면 (+), 앞으로 숙여지면 (-)
        pitch_deg = math.degrees(pitch)
        pitch_rate = gyro[1]                          # Pitch축 회전 각속도
        gyro_norm = np.linalg.norm(gyro)              # 3축 회전 각속도 합성 크기

        dt = self.model.opt.timestep

        # 발끝 지면 접촉(Touchdown) 물리 충돌 검출 (MuJoCo Contact Manifold)
        # 원본 robot.xml에는 발끝 센서가 없으므로 MuJoCo data.contact 및 충돌 반력을 직접 계산
        contact_L = False
        contact_R = False
        touch_L_force = 0.0
        touch_R_force = 0.0
        c_force = np.zeros(6, dtype=np.float64)

        for i in range(data.ncon):
            con = data.contact[i]
            if con.geom1 == self.foot_L_geom_id or con.geom2 == self.foot_L_geom_id:
                contact_L = True
                mujoco.mj_contactForce(self.model, data, i, c_force)
                touch_L_force += float(np.linalg.norm(c_force[:3]))
            if con.geom1 == self.foot_R_geom_id or con.geom2 == self.foot_R_geom_id:
                contact_R = True
                mujoco.mj_contactForce(self.model, data, i, c_force)
                touch_R_force += float(np.linalg.norm(c_force[:3]))

        # robot_state 기본 물리 상태 갱신
        self.robot_state.sim_time = sim_time
        self.robot_state.pos_x = pos_x
        self.robot_state.pos_z = pos_z
        self.robot_state.pitch = pitch
        self.robot_state.pitch_deg = pitch_deg
        self.robot_state.pitch_rate = pitch_rate
        self.robot_state.gyro_norm = gyro_norm
        self.robot_state.touch_L_force = touch_L_force
        self.robot_state.touch_R_force = touch_R_force
        self.robot_state.touch_L = contact_L
        self.robot_state.touch_R = contact_R

        # =========================================================================
        # 2. 수평 상태 및 기울기 추적
        # =========================================================================
        is_horizontal = abs(pitch) <= self.pitch_tolerance
        if is_horizontal:
            self.robot_state.tilt_duration = 0.0
        else:
            self.robot_state.tilt_duration += dt

        # =========================================================================
        # 3. 전도('falling') 감지
        # - 상체 높이 추락(Z < 0.45m), 45° 이상 전도, 또는 3초 이상 지속 기울기 시 감지
        # =========================================================================
        is_overturned = abs(pitch) > math.radians(45.0)
        if sim_time > 0.05:
            should_fall = (pos_z < 0.45) or is_overturned or (self.robot_state.tilt_duration >= self.tilt_max_time)
        else:
            should_fall = is_overturned

        if should_fall and self.robot_state.state != "falling":
            self.robot_state.is_fallen = True
            self.log_fall = True
            self.set_state("falling", f"전도 감지 (Z={pos_z:.2f}m, Pitch={pitch_deg:.1f}°)")

        # =========================================================================
        # 4. FSM 상태 전이 및 능동 자세/발구름 제어
        # 1) landing (0.8초): 양발 지면 안착 및 상체 수평 안정화
        # 2) ready (0.5초): 직립 자세 확립 및 발구름 시퀀스 대기
        # 3) stepping: Raibert 피치 적응형 제자리 발구름(In-place Stepping)
        # =========================================================================
        if self.robot_state.state == "landing":
            self.q_target = self.q_stand.copy()
            self.landing_timer += dt
            # 양발 착지 및 수평 유지 확인 후 0.8초 경과 시 ready 상태로 전이
            if self.landing_timer >= self.landing_duration and (contact_L or contact_R):
                self.set_state("ready", "지면 안착 및 수평 안정화 완료 ➔ 발구름 준비 대기")
                self.ready_timer = 0.0

        elif self.robot_state.state == "ready":
            self.q_target = self.q_stand.copy()
            self.ready_timer += dt
            # 0.5초간 직립 자세를 완벽히 유지한 후 제자리 발구름 시작
            if self.ready_timer >= self.ready_duration:
                self.set_state("stepping", "직립 안정화 완료 ➔ 제자리 발구름(In-place Stepping) 시작!")
                self.stepping_timer = 0.0

        elif self.robot_state.state == "stepping":
            # 직립 자세 기반 Raibert 피치 적응형 제자리 발구름 (In-place Stepping)
            self.stepping_timer += dt
            phi_L = (self.stepping_timer % self.gait_period) / self.gait_period
            phi_R = ((self.stepping_timer + 0.5 * self.gait_period) % self.gait_period) / self.gait_period

            self.q_target = self.q_stand.copy()

            # Raibert 피치 적응형 발 착지 오프셋
            # pitch > 0 (앞으로 넘어짐) -> 유각 발을 전방 착지 (+X)하여 전도 방지 -> hip_L(+), hip_R(-)
            # pitch < 0 (뒤로 넘어짐) -> 유각 발을 후방 착지 (-X)하여 전도 방지 -> hip_L(-), hip_R(+)
            raibert_offset = self.k_raibert * pitch + self.k_raibert_rate * pitch_rate
            raibert_offset = np.clip(raibert_offset, -0.12, 0.12)

            # 교대 발 들어올리기 및 착지각 유지 (Swing & Stance Persistence)
            if phi_L < 0.5:
                # [왼발 유각(Swing) / 오른발 지지(Stance)]
                lift = math.sin(2.0 * math.pi * phi_L) * self.step_height
                self.target_hip_L = self.q_stand[1] + raibert_offset
                self.target_hip_R = self.q_stand[4]
                self.q_target[1] = self.target_hip_L
                self.q_target[4] = self.target_hip_R
                self.q_target[2] += lift
                self.q_target[5] += 0.010           # 오른무릎 살짝 펴서 지탱
            else:
                # [오른발 유각(Swing) / 왼발 지지(Stance)]
                lift = math.sin(2.0 * math.pi * phi_R) * self.step_height
                self.target_hip_R = self.q_stand[4] - raibert_offset
                self.target_hip_L = self.q_stand[1]
                self.q_target[4] = self.target_hip_R
                self.q_target[1] = self.target_hip_L
                self.q_target[5] -= lift
                self.q_target[2] -= 0.010           # 왼무릎 살짝 펴서 지탱

            # 지지발 쪽으로 골반 롤(Roll) 스웨이
            sway = math.sin(2.0 * math.pi * phi_L) * self.roll_sway
            self.q_target[0] -= sway
            self.q_target[3] -= sway

        # =========================================================================
        # 5. 최종 목표 관절 각도 및 PD 토크 연산 (+ 상체 피치 능동 안정화 토크)
        # =========================================================================
        q_des = self.q_target.copy()

        # 관절 PD 토크 연산
        joint_error = q_des - q_act
        torques = self.kp * joint_error - self.kd * v_act

        # 고관절(Hip)에 상체 Pitch 복원 토크 가산:
        # pitch > 0 (전방 숙여짐): hip_L에 양수 토크 인가 ➔ 다리를 앞으로 밀며 반작용으로 상체를 뒤로 기립
        # pitch < 0 (후방 젖혀짐): hip_L에 음수 토크 인가 ➔ 다리를 뒤로 당기며 반작용으로 상체를 앞으로 기립
        tau_pitch = self.kp_pitch * pitch + self.kd_pitch * pitch_rate
        tau_pitch = np.clip(tau_pitch, -15.0, 15.0)
        torques[1] += tau_pitch   # hip_L (+Y 축)
        torques[4] -= tau_pitch   # hip_R (-Y 축, 반대 회전축)

        torques = np.clip(torques, -60.0, 60.0)

        # 텔레메트리 로깅
        self.robot_state.max_joint_error = np.max(np.abs(joint_error))
        self.log_time.append(sim_time)
        self.log_z.append(pos_z)
        self.log_pitch.append(pitch)
        self.log_gyro.append(np.linalg.norm(gyro[:2]))
        self.log_max_error.append(self.robot_state.max_joint_error)
        self.log_state.append(self.robot_state.state)

        return torques

def run_simulation_loop(model, data, controller, viewer=None, max_time=10.0, reset_requested=None, paused=None):
    last_print_time = 0.0
    prev_sim_time = data.time

    # 1.0x 실시간 동기화를 위한 기준 시각
    wall_start = time.perf_counter()
    sim_start = data.time

    print(f"\n{Colors.BOLD}[TEST EXECUTION] Running Spawn Pose Hold Simulation...{Colors.RESET}", flush=True)
    if viewer:
        print(f"  {Colors.CYAN}📺 3D MuJoCo 뷰어가 활성화되었습니다. (Space: 일시정지, 'R' 키 / Backspace / 터미널 Enter: 리셋){Colors.RESET}", flush=True)

    step = 0
    evaluation_done = False

    def do_reset():
        nonlocal evaluation_done, wall_start, sim_start, last_print_time, prev_sim_time, step
        stand_key_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "stand")

        # 뷰어 백그라운드 렌더러 스레드와의 레이스 컨디션 방지를 위해 viewer.lock() 적용
        if viewer:
            with viewer.lock():
                if stand_key_id != -1:
                    mujoco.mj_resetDataKeyframe(model, data, stand_key_id)
                else:
                    mujoco.mj_resetData(model, data)
                    data.qpos[7:13] = controller.q_stand.copy()
                data.time = 0.0
                data.qvel[:] = 0.0
                data.ctrl[:] = 0.0
                mujoco.mj_forward(model, data)
        else:
            if stand_key_id != -1:
                mujoco.mj_resetDataKeyframe(model, data, stand_key_id)
            else:
                mujoco.mj_resetData(model, data)
                data.qpos[7:13] = controller.q_stand.copy()
            data.time = 0.0
            data.qvel[:] = 0.0
            data.ctrl[:] = 0.0
            mujoco.mj_forward(model, data)

        # 제어기 및 robot_state 완전 초기화 (landing 상태로 리셋)
        controller.reset()

        # 루프 제어 타이머 및 변수 완전 재동기화
        evaluation_done = False
        wall_start = time.perf_counter()
        sim_start = 0.0
        prev_sim_time = 0.0
        step = 0
        if viewer:
            viewer.sync()

        rs = controller.robot_state
        print(f"\n  {Colors.BOLD}{Colors.YELLOW}↺ [RESET 완료] 시뮬레이션 및 로봇 상태가 초기 스폰 상태(robot_state='landing')로 완벽히 재동기화되었습니다.{Colors.RESET}", flush=True)
        print(f"  * [ 0.00s] robot_state: [{rs.state:^11}] (지면안착: {controller.landing_timer:.1f}s/0.8s) | Pitch={rs.pitch_deg:+5.1f}° | 높이 Z={rs.pos_z:5.3f}m | 관절 오차=0.0000 rad\n", flush=True)
        last_print_time = 0.0

    # 시뮬레이션 시작 즉시 0.00s 초기 landing 상태 1회 강제 출력
    rs_init = controller.robot_state
    print(f"  * [ 0.00s] robot_state: [{rs_init.state:^11}] (지면안착: {controller.landing_timer:.1f}s/0.8s) | Pitch={rs_init.pitch_deg:+5.1f}° | 높이 Z={rs_init.pos_z:5.3f}m | 관절 오차=0.0000 rad", flush=True)

    while True:
        if viewer and not viewer.is_running():
            print(f"  * 사용자에 의해 뷰어 창이 닫혔습니다.", flush=True)
            break

        # [핵심 1] 일시정지 (Space) 처리
        if paused and paused[0]:
            if viewer:
                viewer.sync()
            time.sleep(0.02)
            if reset_requested and reset_requested[0]:
                reset_requested[0] = False
                do_reset()
            continue

        # [핵심 2] 뷰어 및 터미널 Reset 감지
        is_reset = False
        if reset_requested and reset_requested[0]:
            is_reset = True
            reset_requested[0] = False
        # 뷰어 UI Reset 버튼 클릭 등으로 data.time이 되돌려진 경우 감지
        elif prev_sim_time > 0.05 and (data.time < prev_sim_time - 0.01 or data.time == 0.0):
            is_reset = True
        elif data.time <= 0.001 and (time.perf_counter() - wall_start) > 0.1 and controller.robot_state.state != "landing":
            is_reset = True

        # 터미널 창 포커스 시 Enter 또는 'r' 입력 감지 (비차단)
        if not is_reset:
            try:
                import select
                if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                    line = sys.stdin.readline().strip().lower()
                    if line in ('r', 'reset', '', 'ㄱ', 'restart'):
                        print(f"\n  {Colors.BOLD}{Colors.YELLOW}↺ [터미널 입력 감지] Reset 요청됨!{Colors.RESET}", flush=True)
                        is_reset = True
            except Exception:
                pass

        if is_reset:
            do_reset()

        # ---------------------------------------------------------------------
        # [1.0x 완벽 실시간 동기화]: 실제 벽시계 시간(Wall time)과 물리 시간 1:1 일치
        # ---------------------------------------------------------------------
        wall_elapsed = time.perf_counter() - wall_start
        
        # 렌더링 한 프레임 동안 실제 흘러간 시간만큼 물리 스텝(dt=0.001s)을 전진
        # (렌더링 vsync 지연에 발목 잡히지 않고 물리 엔진이 현실 시간을 완벽히 추종)
        step_count = 0
        max_substeps = 40
        while (data.time - sim_start) < wall_elapsed and step_count < max_substeps:
            # 1. 제어 토크 인가
            torques = controller.compute_torques(data)
            for i, act_id in enumerate(controller.act_ids):
                data.ctrl[act_id] = torques[i]

            # 2. 물리 1 스텝 전진
            mujoco.mj_step(model, data)
            step_count += 1
            step += 1

        # 물리 계산이 실제 시간보다 앞선 경우 잉여 시간만큼 미세 대기
        time_ahead = (data.time - sim_start) - (time.perf_counter() - wall_start)
        if time_ahead > 0.001:
            time.sleep(time_ahead)

        # 3. 뷰어 화면 동기화 (프레임당 1회 깔끔하게 렌더링)
        if viewer:
            time_before_sync = data.time
            viewer.sync()
            # 뷰어 GUI [Reset] 버튼 클릭 감지:
            # sync 전 data.time이 0.05s 이상이었는데 sync 후 data.time이 되돌려졌거나 0.0s가 되었다면 즉시 리셋 플래그 활성화
            if time_before_sync > 0.05 and (data.time < time_before_sync - 0.01 or data.time == 0.0):
                reset_requested[0] = True

        # 4. 실시간 텍스트 상태 출력 (0.5초 주기)
        sim_time = data.time
        if (sim_time - last_print_time) >= 0.5:
            rs = controller.robot_state
            if rs.state == "landing":
                state_detail = f"지면안착: {controller.landing_timer:3.1f}s/0.8s (L={rs.touch_L_force:.1f}N, R={rs.touch_R_force:.1f}N)"
            elif rs.state == "ready":
                state_detail = f"직립준비: {controller.ready_timer:3.1f}s/0.5s"
            elif rs.state == "stepping":
                touch_str = f"L:{'ON' if rs.touch_L else '--'} R:{'ON' if rs.touch_R else '--'}"
                state_detail = f"발구름유지: {controller.stepping_timer:3.1f}s ({touch_str})"
            elif rs.state == "falling":
                state_detail = f"전도발생: {rs.tilt_duration:3.1f}s"
            else:
                state_detail = "정상유지"

            print(f"  * [{sim_time:5.2f}s] robot_state: [{rs.state:^11}] ({state_detail}) | Pitch={rs.pitch_deg:+5.1f}° | 높이 Z={rs.pos_z:5.3f}m | 관절 오차={rs.max_joint_error:5.4f} rad", flush=True)
            last_print_time = sim_time

        if controller.robot_state.state == "falling" and not evaluation_done:
            rs = controller.robot_state
            print(f"  {Colors.RED}✗ [{sim_time:.2f}s] 로봇 수평 불량/전도 감지! robot_state='falling' (Pitch: {rs.pitch_deg:.1f}°, 지속시간: {rs.tilt_duration:.2f}s){Colors.RESET}")
            evaluation_done = True
            evaluate_results(controller)
            if not viewer:
                break

        # 헤드리스 모드 종료 조건
        if not viewer and data.time >= max_time:
            if not evaluation_done:
                evaluate_results(controller)
                export_snapshot(model, data)
            break

        prev_sim_time = data.time

    return controller, data

def evaluate_results(controller):
    rs = controller.robot_state
    print(f"\n{Colors.BOLD}[VERIFICATION RESULTS] Spawn & State Transition Evaluation{Colors.RESET}")
    print(f"  * 최종 robot_state: [{rs.state}]")
    if rs.state == "falling" or controller.log_fall:
        print(f"  {Colors.RED}✗ [Check 1] 상체 수평 유지 및 전도 여부: 넘어짐 감지 ('falling') [FAIL]{Colors.RESET}")
        return False
    else:
        print(f"  {Colors.GREEN}✓ [Check 1] 상체 수평 유지 및 전도 여부: 정상 기립 및 제자리 발구름 진입 ('{rs.state}') [PASS]{Colors.RESET}")

    final_z = controller.log_z[-1] if len(controller.log_z) > 0 else 0.0
    print(f"  * 최종 베이스 높이: z = {final_z:.3f} m")

    final_pitch = controller.log_pitch[-1] if len(controller.log_pitch) > 0 else 0.0
    print(f"  * 최종 상체 Pitch: {math.degrees(final_pitch):+.2f}° ({final_pitch:+.4f} rad)")

    final_err = controller.log_max_error[-1] if len(controller.log_max_error) > 0 else 0.0
    print(f"  * 최종 관절 최대 오차: {final_err:.4f} rad ({math.degrees(final_err):.2f}°)")

    err_pass = (final_err <= 0.08)
    if err_pass:
        print(f"  {Colors.GREEN}✓ [Check 2] 관절 자세 추종 정밀도 합격 (오차 <= 0.08 rad) [PASS]{Colors.RESET}")
    else:
        print(f"  {Colors.YELLOW}⚠ [Check 2] 관절 자세 오차 존재 (> 0.08 rad){Colors.RESET}")

    return not controller.log_fall and err_pass

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
    print(f"{Colors.BOLD}{Colors.CYAN}Phase 01-U01: Tron1 Spawn Pose Hold & Stand-Up Test{Colors.RESET}")
    print(f"{Colors.BOLD}{Colors.CYAN}============================================================{Colors.RESET}")
    print(f"  * Target XML : {args.xml}")
    print(f"  * Max Time   : {args.max_time} s")

    if not os.path.exists(args.xml):
        print(f"{Colors.RED}✗ XML 파일을 찾을 수 없습니다: {args.xml}{Colors.RESET}")
        return 1

    model = mujoco.MjModel.from_xml_path(args.xml)
    data = mujoco.MjData(model)

    # XML 내 'stand' 키프레임 존재 여부 확인 및 자동 로드
    stand_key_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "stand")
    if stand_key_id != -1:
        mujoco.mj_resetDataKeyframe(model, data, stand_key_id)
        print(f"  {Colors.GREEN}✓ XML 'stand' 키프레임(초기 스폰 자세)을 성공적으로 로드했습니다.{Colors.RESET}")
    else:
        mujoco.mj_resetData(model, data)

    controller = Tron1PoseHoldController(model)

    # 스폰 시 직립 자세로 지면 안착 (키프레임이 없을 경우 fallback)
    if stand_key_id == -1:
        data.qpos[7:13] = controller.q_stand.copy()
    mujoco.mj_forward(model, data)

    print(f"  * 목표 직립 자세 (Stand q) : {np.round(controller.q_stand, 3)}")

    # 뷰어 실행 모드 분기 (기본값: 3D 창 표시 + 텍스트 동시 출력)
    reset_requested = [False]
    paused = [False]

    # 터미널 창 포커스 시 Enter 또는 'r' 입력을 즉시 캡처하는 백그라운드 리스너 스레드
    def start_terminal_listener():
        def listener():
            while True:
                try:
                    line = sys.stdin.readline()
                    if not line:
                        break
                    text = line.strip().lower()
                    if text in ('', 'r', 'reset', 'ㄱ', 'restart', '0'):
                        reset_requested[0] = True
                        print(f"\n  {Colors.BOLD}{Colors.YELLOW}↺ [터미널 입력 감지: '{text}'] Reset 요청됨!{Colors.RESET}", flush=True)
                    elif text in ('p', 'pause', ' '):
                        paused[0] = not paused[0]
                        state_str = "일시정지 (PAUSED)" if paused[0] else "재개 (RESUMED)"
                        print(f"\n  {Colors.CYAN}⏸ [시뮬레이션 {state_str}]{Colors.RESET}", flush=True)
                except Exception:
                    break
        t = threading.Thread(target=listener, daemon=True)
        t.start()

    start_terminal_listener()

    if not args.headless:
        def key_callback(keycode):
            # 'r', 'R' (82, 114), Backspace (259), Delete (261), Enter (257)
            if keycode in (82, 114, 259, 261, 257, ord('r'), ord('R')):
                reset_requested[0] = True
                print(f"\n  {Colors.BOLD}{Colors.YELLOW}↺ [뷰어 키 입력 감지: keycode={keycode}] Reset 요청됨!{Colors.RESET}", flush=True)
            elif keycode == 32:  # Spacebar (Pause/Resume key)
                paused[0] = not paused[0]
                state_str = "일시정지 (PAUSED)" if paused[0] else "재개 (RESUMED)"
                print(f"\n  {Colors.CYAN}⏸ [시뮬레이션 {state_str}]{Colors.RESET}", flush=True)
            else:
                key_name = chr(keycode) if 32 <= keycode <= 126 else str(keycode)
                print(f"  [Viewer 키 입력 감지: '{key_name}' (code={keycode})] (리셋: 'R' 키, 'Backspace', 또는 터미널 Enter)", flush=True)

        try:
            with mujoco.viewer.launch_passive(model, data, key_callback=key_callback) as viewer:
                viewer.opt.geomgroup[0] = 1
                viewer.opt.geomgroup[1] = 1
                viewer.opt.geomgroup[2] = 1
                viewer.opt.geomgroup[3] = 1
                controller, final_data = run_simulation_loop(
                    model, data, controller, viewer=viewer, max_time=args.max_time,
                    reset_requested=reset_requested, paused=paused
                )
        except Exception as e:
            print(f"  {Colors.YELLOW}⚠ GUI 뷰어 실행 실패 ({e}) -> 텍스트 전용 모드로 전환{Colors.RESET}", flush=True)
            controller, final_data = run_simulation_loop(model, data, controller, viewer=None, max_time=args.max_time, reset_requested=reset_requested, paused=paused)
    else:
        controller, final_data = run_simulation_loop(model, data, controller, viewer=None, max_time=args.max_time, reset_requested=reset_requested, paused=paused)

    return 0

if __name__ == "__main__":
    sys.exit(main())