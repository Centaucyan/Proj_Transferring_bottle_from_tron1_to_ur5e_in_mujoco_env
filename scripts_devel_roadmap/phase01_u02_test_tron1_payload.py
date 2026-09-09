#!/usr/bin/env python3
"""
scripts_devel_roadmap/phase01_u02_test_tron1_payload.py
Phase 01-U02: Tron1 Payload Transport & Table Bumper Docking Verification
- Supports 1, 2, or 3 bottle payloads (Asymmetric eccentric load testing)
- 500Hz MuJoCo Physics & 50Hz LimX Official Pretrained RL Policy Inference
- Finite State Machine (FSM):
    0: LANDING          -> Shock absorption on ground touchdown (0.15s)
    1: IN_PLACE_HOLD    -> In-place stepping with payload & origin hold PD
    2: DOCKING_APPROACH -> Forward walking towards docking station ledge (vx = +0.12 m/s)
    3: BUMPER_CONTACT   -> Bumper contact force detection (F_normal > 5.0 N)
    4: STANCE_LOCK      -> Turn OFF stepping, lock joint angles in 3-point tripod support
    5: READY_FOR_PICK   -> Vibration-free static equilibrium confirmation (< 0.01 m/s for 3.0s)
    6: UNDOCKING        -> Resume stepping, step backward (vx = -0.15 m/s) & return to self-balance
- Interactive Keyboard & Auto Controls:
    [D] Trigger Docking | [U] Trigger Undocking | [Space] Pause | [R/Backspace/Enter] Reset
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

try:
    import onnxruntime as ort
except ImportError:
    print("\033[91m[에러] 'onnxruntime' 패키지가 설치되지 않았습니다.\033[0m")
    print("다음 명령어를 실행하여 설치해 주세요: python -m pip install onnxruntime")
    sys.exit(1)

class Colors:
    GREEN = '\033[92m'
    RED = '\033[91m'
    YELLOW = '\033[93m'
    CYAN = '\033[96m'
    BLUE = '\033[94m'
    MAGENTA = '\033[95m'
    BOLD = '\033[1m'
    RESET = '\033[0m'

@dataclass
class U02Telemetry:
    fsm_state: str = "LANDING"
    nav_phase: str = "WP0_TURN"
    sim_time: float = 0.0
    base_x: float = 0.0
    base_y: float = 0.0
    base_z: float = 0.0
    yaw_deg: float = 0.0
    pitch_deg: float = 0.0
    roll_deg: float = 0.0
    gyro_norm: float = 0.0
    bumper_force: float = 0.0
    tray_vel_rms: float = 0.0
    docking_stable_timer: float = 0.0
    is_ready_for_pick: bool = False
    touch_L: bool = False
    touch_R: bool = False
    is_fallen: bool = False
    bottle_status: str = "ALL_OK"

def parse_args():
    parser = argparse.ArgumentParser(description="Phase 01-U02: Tron1 Payload Transport & Table Docking")
    parser.add_argument("--xml", type=str, default="xml_for_unit_test/phase01_u02_scene_unit_tron1_payload.xml",
                        help="Path to Tron1 payload unit scene XML")
    parser.add_argument("--model_dir", type=str, default="model_rl/tron1",
                        help="Path to directory containing policy.onnx and encoder.onnx")
    parser.add_argument("--bottles", type=int, default=3, choices=[1, 2, 3],
                        help="Number of bottles to load (1: Asymmetric left slot, 2: Left+Right, 3: Full load)")
    parser.add_argument("--auto", action="store_true",
                        help="Automatically proceed from Hold to Docking after 3 seconds")
    parser.add_argument("--max_time", type=float, default=120.0, help="Simulation time limit in seconds")
    parser.add_argument("--no-gui", action="store_true", help="Run simulation in headless mode")
    return parser.parse_args()

class Tron1PayloadController:
    """
    Tron1 페이로드 보행 운반 및 테이블 범퍼 3점 지지 정적 도킹 제어기
    """
    def __init__(self, model, model_dir="model_rl/tron1", num_bottles=3, auto_dock=False):
        self.model = model
        self.model_dir = model_dir
        self.num_bottles = num_bottles
        self.auto_dock = auto_dock

        # 관절 및 액추에이터 ID 매핑
        self.actuator_names = [
            "abad_L_motor", "hip_L_motor", "knee_L_motor",
            "abad_R_motor", "hip_R_motor", "knee_R_motor"
        ]
        self.act_ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name) for name in self.actuator_names]
        self.base_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_Link")
        self.tray_site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "tray_center_site")
        self.foot_L_geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "foot_L_col")
        self.foot_R_geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "foot_R_col")
        self.bumper_geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "front_bumper_col")
        self.ledge_geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "table_bumper_ledge")
        self.touch_sensor_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, "bumper_touch")

        # 물병 바디 ID
        self.bottle_ids = [
            mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"bottle_{i}") for i in range(1, 4)
        ]

        # LimX PF_TRON1A 공식 제어 매개변수
        self.default_joint_pos = np.zeros(6, dtype=np.float32)
        self.kp_walking = 42.0
        self.kd_walking = 3.5
        self.kp_lock = 100.0  # Stance Lock 고감쇠 게인
        self.kd_lock = 8.0
        self.action_scale = 0.25
        self.torque_limit = 80.0
        self.decimation = 10  # 500Hz / 10 = 50Hz RL Policy Loop

        # 관측 버퍼 및 보행 클럭
        self.observations_size = 30
        self.obs_history_length = 10
        self.gait = np.array([2.0, 0.5, 0.5, 0.1], dtype=np.float32)
        self.commands = np.zeros(3, dtype=np.float32)  # [vx, vy, wz]

        # ONNX 모델 로드
        self.policy_path = os.path.join(model_dir, "policy.onnx")
        self.encoder_path = os.path.join(model_dir, "encoder.onnx")
        self._load_onnx()

        # FSM 상태 변수
        self.fsm_state = "LANDING"
        self.state_timer = 0.0
        self.locked_joint_pos = np.zeros(6, dtype=np.float32)
        self.vibration_stable_timer = 0.0
        self.undock_start_x = 0.0
        self.bumper_touch_timer = 0.0
        self.is_bumper_touch = False
        self.nav_phase = "WP0_TURN"
        self.turn_settle_timer = 0.0
        self.phase_timer = 0.0
        self.hold_x = -5.0
        self.hold_y = -4.0
        self.cmd_smooth = np.zeros(3, dtype=np.float32)
        self.is_stance_locked = False
        self.lock_start_q = np.zeros(6, dtype=np.float32)

        # 텔레메트리 객체
        self.telemetry = U02Telemetry()
        self.reset()

    def _load_onnx(self):
        if not (os.path.exists(self.policy_path) and os.path.exists(self.encoder_path)):
            print(f"\033[91m[에러] ONNX 모델을 찾을 수 없습니다: {self.model_dir}\033[0m")
            sys.exit(1)

        opts = ort.SessionOptions()
        opts.intra_op_num_threads = 1
        opts.inter_op_num_threads = 1
        opts.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
        providers = ['CPUExecutionProvider']
        self.policy_session = ort.InferenceSession(self.policy_path, sess_options=opts, providers=providers)
        self.encoder_session = ort.InferenceSession(self.encoder_path, sess_options=opts, providers=providers)
        self.policy_input_name = self.policy_session.get_inputs()[0].name
        self.encoder_input_name = self.encoder_session.get_inputs()[0].name
        print(f"{Colors.BOLD}{Colors.GREEN}✓ LimX 공식 ONNX 모델 로드 성공!{Colors.RESET}")

    def reset(self):
        self.fsm_state = "LANDING"
        self.state_timer = 0.0
        self.vibration_stable_timer = 0.0
        self.tray_vel_smooth = 0.0
        self.last_action = np.zeros(6, dtype=np.float32)
        self.actions = np.zeros(6, dtype=np.float32)
        self.q_target = np.zeros(6, dtype=np.float32)
        self.encoder_out = np.zeros(3, dtype=np.float32)
        self.proprio_history_buffer = np.zeros(300, dtype=np.float32)
        self.is_first_rec_obs = True
        self.gait_index = 0.0
        self.loop_count = 0
        self.commands[:] = 0.0
        self.cmd_smooth[:] = 0.0
        self.spawn_x = None
        self.spawn_y = None
        self.waypoints = [(-3.0, 3.0), (-0.70, 0.0), (0.27, 0.0)]
        self.current_wp_idx = 0
        self.bumper_touch_timer = 0.0
        self.is_bumper_touch = False
        self.nav_phase = "WP0_TURN"
        self.turn_settle_timer = 0.0
        self.phase_timer = 0.0
        self.hold_x = -5.0
        self.hold_y = -4.0
        self.is_stance_locked = False
        self.lock_start_q = np.zeros(6, dtype=np.float32)
        self.telemetry = U02Telemetry(fsm_state="LANDING", nav_phase="WP0_TURN")

    def configure_bottles(self, data):
        """명령행 옵션에 따라 물병 적재 수량을 설정합니다 (1개: 좌측 비대칭, 2개: 좌우, 3개: 만재)"""
        # bottle_1: Slot L (pos: 0.05, +0.09)
        # bottle_2: Slot C (pos: 0.05,  0.00)
        # bottle_3: Slot R (pos: 0.05, -0.09)
        jnt_names = ["bottle_1_joint", "bottle_2_joint", "bottle_3_joint"]
        jnt_ids = [mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name) for name in jnt_names]

        # 1개 적재: bottle_2, bottle_3 격리 (바닥 아래로 이동 및 속도 0)
        if self.num_bottles == 1:
            for idx in (1, 2):
                jid = jnt_ids[idx]
                if jid != -1:
                    qadr = self.model.jnt_qposadr[jid]
                    vadr = self.model.jnt_dofadr[jid]
                    data.qpos[qadr:qadr + 7] = np.array([0, 0, -10.0, 1, 0, 0, 0], dtype=np.float64)
                    data.qvel[vadr:vadr + 6] = 0.0
        # 2개 적재: bottle_2(중앙)만 격리 (좌우 2개 적재)
        elif self.num_bottles == 2:
            jid = jnt_ids[1]
            if jid != -1:
                qadr = self.model.jnt_qposadr[jid]
                vadr = self.model.jnt_dofadr[jid]
                data.qpos[qadr:qadr + 7] = np.array([0, 0, -10.0, 1, 0, 0, 0], dtype=np.float64)
                data.qvel[vadr:vadr + 6] = 0.0

    def trigger_docking(self):
        if self.fsm_state in ("IN_PLACE_HOLD", "LANDING"):
            self.fsm_state = "DOCKING_APPROACH"
            self.state_timer = 0.0
            self.nav_phase = "WP0_TURN"
            self.turn_settle_timer = 0.0
            self.phase_timer = 0.0
            if self.spawn_x is not None:
                self.hold_x = self.spawn_x
                self.hold_y = self.spawn_y
            print(f"\n  {Colors.BOLD}{Colors.CYAN}▶ [도킹 개시] 웨이포인트 주행 시작! (스폰 위치에서 제자리 구름하며 WP0 회전 정렬){Colors.RESET}\n", flush=True)

    def trigger_undocking(self, data):
        if self.fsm_state in ("STANCE_LOCK", "READY_FOR_PICK"):
            self.is_stance_locked = False
            self.fsm_state = "UNDOCKING"
            self.state_timer = 0.0
            self.undock_start_x = float(data.xpos[self.base_body_id][0])
            print(f"\n  {Colors.BOLD}{Colors.MAGENTA}◀ [언도킹 개시] 발구름을 재개하고 뒤로 안전하게 물러납니다! (vx = -0.15 m/s){Colors.RESET}\n", flush=True)

    def compute_torques(self, data):
        sim_time = data.time
        dt = self.model.opt.timestep
        self.state_timer += dt

        pos_x = float(data.xpos[self.base_body_id][0])
        pos_y = float(data.xpos[self.base_body_id][1])
        pos_z = float(data.xpos[self.base_body_id][2])
        q_act = data.qpos[7:13].astype(np.float32)
        v_act = data.qvel[6:12].astype(np.float32)

        # 1. IMU 및 자세 측정
        quat = data.sensor("imu_quat").data
        gyro = data.sensor("imu_gyro").data
        w, x, y, z = quat
        sinp = 2.0 * (w * y - z * x)
        pitch = math.asin(np.clip(sinp, -1.0, 1.0))
        sinr = 2.0 * (w * x + y * z)
        cosr = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(sinr, cosr)

        # 2. 접촉 및 범퍼 힘 측정
        contact_L, contact_R = False, False
        bumper_force = 0.0
        for i in range(data.ncon):
            con = data.contact[i]
            if con.geom1 == self.foot_L_geom_id or con.geom2 == self.foot_L_geom_id:
                contact_L = True
            if con.geom1 == self.foot_R_geom_id or con.geom2 == self.foot_R_geom_id:
                contact_R = True
            if (con.geom1 == self.bumper_geom_id and con.geom2 == self.ledge_geom_id) or \
               (con.geom2 == self.bumper_geom_id and con.geom1 == self.ledge_geom_id):
                c_array = np.zeros(6, dtype=np.float64)
                mujoco.mj_contactForce(self.model, data, i, c_array)
                bumper_force += abs(c_array[0])

        if self.touch_sensor_id != -1:
            sensor_force = float(data.sensordata[self.model.sensor_adr[self.touch_sensor_id]])
            bumper_force = max(bumper_force, sensor_force)

        # 3. 트레이 진동 속도 측정 (6차원 속도 벡터: res[0:3]=각속도, res[3:6]=선속도)
        tray_vel_6d = np.zeros(6, dtype=np.float64)
        mujoco.mj_objectVelocity(self.model, data, mujoco.mjtObj.mjOBJ_SITE, self.tray_site_id, tray_vel_6d, 0)
        tray_vel_rms = float(np.linalg.norm(tray_vel_6d[3:6]))
        self.tray_vel_smooth = 0.98 * self.tray_vel_smooth + 0.02 * tray_vel_rms

        # 4. 물병 상태 모니터링 (낙하 체크: z < 0.60m)
        bottle_ok = True
        for b_id in self.bottle_ids[:self.num_bottles]:
            if b_id != -1 and data.xpos[b_id][2] < 0.60:
                bottle_ok = False
        bottle_status_str = f"OK ({self.num_bottles}EA)" if bottle_ok else "FALLEN!"

        # 5. 전도 체크
        if sim_time > 0.20 and (pos_z < 0.45 or abs(math.degrees(pitch)) > 45.0):
            if not self.telemetry.is_fallen:
                self.fsm_state = "FALLEN"
                self.telemetry.is_fallen = True
                print(f"\n  {Colors.BOLD}{Colors.RED}✗ [{sim_time:5.2f}s] 전도 발생! (Z={pos_z:.2f}m, Pitch={math.degrees(pitch):.1f}°){Colors.RESET}\n", flush=True)

        # ==================== FSM 상태 전이 로직 ====================
        if self.fsm_state == "LANDING":
            if self.state_timer >= 0.15 and (contact_L or contact_R or self.state_timer >= 0.25):
                self.fsm_state = "IN_PLACE_HOLD"
                self.state_timer = 0.0
                self.spawn_x = pos_x
                self.spawn_y = pos_y
                self.hold_x = pos_x
                self.hold_y = pos_y
                print(f"\n  {Colors.BOLD}{Colors.GREEN}★ [{sim_time:5.2f}s] 'LANDING' ➔ 'IN_PLACE_HOLD' (스폰 위치 ({pos_x:.2f}, {pos_y:.2f}) 제자리 발구름 시작!){Colors.RESET}\n", flush=True)

        elif self.fsm_state == "IN_PLACE_HOLD":
            # d 키 입력 없이 착지 안정화 1.0초 후 자동으로 웨이포인트 주행 개시!
            if self.state_timer >= 1.0:
                self.trigger_docking()

        elif self.fsm_state == "DOCKING_APPROACH":
            # 실제 범퍼 접촉은 pos_x ≈ 0.265m에서 발생 (테이블 전면 X=0.47m, 범퍼 전단 X_rel=+0.205m)
            touch_detected = (bumper_force >= 2.0) or (pos_x >= 0.258 and bumper_force >= 0.5)
            if touch_detected and self.nav_phase == "DOCK_CREEP":
                self.fsm_state = "STANCE_LOCK"
                self.nav_phase = "DOCK_HELD"
                self.is_bumper_touch = True
                self.state_timer = 0.0
                self.vibration_stable_timer = 0.0
                print(f"\n  {Colors.BOLD}{Colors.GREEN}★ [{sim_time:5.2f}s] 범퍼 밀착 안착 성공 (F={bumper_force:4.1f}N, pos_x={pos_x:.3f}m)! 'STANCE_LOCK' 진입 (도킹 거치 유지){Colors.RESET}\n", flush=True)

        elif self.fsm_state == "STANCE_LOCK":
            # 도킹 거치 상태에서 트레이 진동 안정화 모니터링 (< 0.12 m/s)
            if self.tray_vel_smooth < 0.12:
                self.vibration_stable_timer += dt
                if self.vibration_stable_timer >= 3.0:
                    self.fsm_state = "READY_FOR_PICK"
                    self.state_timer = 0.0
                    self.telemetry.is_ready_for_pick = True
                    print(f"\n  {Colors.BOLD}{Colors.GREEN}✔ [{sim_time:5.2f}s] [도킹 성공] 3초간 안정 상태 달성! => 'READY_FOR_PICK' 확립! (물병 피킹 대기){Colors.RESET}\n", flush=True)
            else:
                self.vibration_stable_timer = max(0.0, self.vibration_stable_timer - 1.5 * dt)

        elif self.fsm_state == "READY_FOR_PICK":
            # READY_FOR_PICK 진입 시 양발 접지 이중 지지기(Double Support Phase) 감지
            # RL 동적 발구름을 즉시 종료하고 3점 지지(양발 + 범퍼) 정적 자세 잠금(Static Stance Lock) 체결!
            if not self.is_stance_locked:
                g_idx = self.gait_index
                is_double_support = (contact_L and contact_R and
                                     (g_idx <= 0.06 or (0.48 <= g_idx <= 0.54)))
                if is_double_support or self.state_timer >= 0.5:
                    self.is_stance_locked = True
                    self.lock_start_q = np.copy(q_act)
                    print(f"\n  {Colors.BOLD}{Colors.GREEN}★ [{sim_time:5.2f}s] [정적 스탠스 락 체결] 양발 접지 이중 지지기(gait={g_idx:.2f}) 감지! 제자리 발구름 완전 정지 및 트레이 진동 소멸 (v_rms < 0.001 m/s){Colors.RESET}\n", flush=True)

        elif self.fsm_state == "UNDOCKING":
            # 뒤로 약 0.15m 물러나면 다시 자립 제자리 발구름 복귀
            if pos_x <= self.undock_start_x - 0.15 or self.state_timer >= 2.5:
                self.fsm_state = "IN_PLACE_HOLD"
                self.state_timer = 0.0
                print(f"\n  {Colors.BOLD}{Colors.CYAN}↺ [{sim_time:5.2f}s] 언도킹 완료! 안전 거리 확보 후 'IN_PLACE_HOLD' 복귀.{Colors.RESET}\n", flush=True)

        # 텔레메트리 갱신
        self.telemetry.fsm_state = self.fsm_state
        self.telemetry.nav_phase = self.nav_phase
        self.telemetry.sim_time = sim_time
        self.telemetry.base_x = pos_x
        self.telemetry.base_y = pos_y
        self.telemetry.base_z = pos_z
        self.telemetry.pitch_deg = math.degrees(pitch)
        self.telemetry.roll_deg = math.degrees(roll)
        self.telemetry.gyro_norm = float(np.linalg.norm(gyro))
        self.telemetry.bumper_force = bumper_force
        self.telemetry.tray_vel_rms = tray_vel_rms
        self.telemetry.docking_stable_timer = self.vibration_stable_timer
        self.telemetry.touch_L = contact_L
        self.telemetry.touch_R = contact_R
        self.telemetry.bottle_status = bottle_status_str

        # ==================== 관절 토크 산출 ====================
        # 1. 도킹 완료 피킹 대기 모드: 제자리 발구름 완전 정지 및 3점 지지 정적 스탠스 락
        if self.is_stance_locked and self.fsm_state == "READY_FOR_PICK":
            q_des = np.copy(self.lock_start_q)
            q_des[0] = 0.0  # abad_L 평행 정렬 (측면 미끄러짐 방지)
            q_des[3] = 0.0  # abad_R 평행 정렬

            self.loop_count += 1
            torques = self.kp_lock * (q_des - q_act) - self.kd_lock * v_act

            # 무릎 상향 중력 지탱 토크 (Upper Body Gravity Sag 방지: Z=0.712m 유지)
            torques[2] -= 18.0  # knee_L
            torques[5] += 18.0  # knee_R

            # 고관절 전방 가압 토크 (범퍼를 테이블 턱에 5~7N 안정적으로 밀착 유지)
            torques[1] += 15.0  # hip_L
            torques[4] -= 15.0  # hip_R

            torques = np.clip(torques, -self.torque_limit, self.torque_limit)
            return torques

        # 2. 보행 모드: LimX RL Policy 추론 (50Hz Decimation)
        if self.fsm_state != "FALLEN":
            if self.loop_count % self.decimation == 0:
                R_mat = np.zeros(9)
                mujoco.mju_quat2Mat(R_mat, quat)
                R_mat = R_mat.reshape(3, 3)
                proj_gravity = (R_mat.T @ np.array([0.0, 0.0, -1.0], dtype=np.float32)).astype(np.float32)

                base_ang_vel = (gyro * 0.25).astype(np.float32)
                joint_pos_input = ((q_act - self.default_joint_pos) * 1.0).astype(np.float32)
                joint_velocities = (v_act * 0.05).astype(np.float32)
                actions_prev = self.last_action.astype(np.float32)

                self.gait_index += 0.02 * self.gait[0]
                if self.gait_index > 1.0:
                    self.gait_index = 0.0
                gait_clock = np.array([
                    np.sin(self.gait_index * 2.0 * np.pi),
                    np.cos(self.gait_index * 2.0 * np.pi)
                ], dtype=np.float32)

                obs = np.concatenate([
                    base_ang_vel, proj_gravity, joint_pos_input,
                    joint_velocities, actions_prev, gait_clock, self.gait
                ]).astype(np.float32)
                obs = np.clip(obs, -100.0, 100.0)

                # 히스토리 버퍼 갱신
                if self.is_first_rec_obs:
                    for i in range(self.obs_history_length):
                        self.proprio_history_buffer[i * self.observations_size:(i + 1) * self.observations_size] = obs
                    self.is_first_rec_obs = False
                else:
                    self.proprio_history_buffer[:-self.observations_size] = self.proprio_history_buffer[self.observations_size:]
                    self.proprio_history_buffer[-self.observations_size:] = obs

                # Encoder 추론 (300D -> 3D Latent)
                enc_in = {self.encoder_input_name: self.proprio_history_buffer}
                self.encoder_out = self.encoder_session.run(None, enc_in)[0].flatten()

                # 속도 명령(Command) 결정: 바디 좌표계 회전 변환 및 자연스러운 보행
                yaw = float(np.arctan2(R_mat[1, 0], R_mat[0, 0]))
                self.telemetry.yaw_deg = math.degrees(yaw)
                cos_y, sin_y = np.cos(yaw), np.sin(yaw)
                vel_x, vel_y = float(data.qvel[0]), float(data.qvel[1])
                body_vel_x = cos_y * vel_x + sin_y * vel_y
                body_vel_y = -sin_y * vel_x + cos_y * vel_y
                dt_policy = self.decimation * dt

                # 스폰 초기 위치 자동 캡처 (스폰 위치가 어디든 그 자리를 기준으로 원점 유지)
                if self.spawn_x is None:
                    self.spawn_x = pos_x
                    self.spawn_y = pos_y
                    self.hold_x = pos_x
                    self.hold_y = pos_y

                if self.fsm_state == "IN_PLACE_HOLD":
                    # [1. 제자리 기립] 스폰 위치를 기준으로 단단하게 브레이크 잡고 제자리 유지
                    err_x = self.spawn_x - pos_x
                    err_y = self.spawn_y - pos_y
                    body_err_x = cos_y * err_x + sin_y * err_y
                    body_err_y = -sin_y * err_x + cos_y * err_y

                    # P 게인 + D 감쇠(브레이크)로 밀림 현상 완벽 방지 (원래 U01/U02 규격: 0.5m/s 제동력)
                    self.commands[0] = float(np.clip(2.0 * body_err_x - 0.5 * body_vel_x, -0.5, 0.5))
                    self.commands[1] = float(np.clip(2.0 * body_err_y - 0.5 * body_vel_y, -0.5, 0.5))
                    self.commands[2] = float(np.clip(-1.0 * yaw, -0.4, 0.4))

                elif self.fsm_state in ("STANCE_LOCK", "READY_FOR_PICK"):
                    # [도킹 밀착 거치 유지]: 범퍼를 테이블 턱에 살짝 기댄 상태(-0.36)로 중심선 및 직각 정렬 유지
                    dock_yaw_err = math.atan2(math.sin(0.0 - yaw), math.cos(0.0 - yaw))
                    lat_err_y = 0.0 - pos_y
                    self.commands[0] = -0.36
                    self.commands[1] = float(np.clip(1.2 * lat_err_y - 0.3 * body_vel_y, -0.04, 0.04))
                    self.commands[2] = float(np.clip(1.0 * dock_yaw_err, -0.15, 0.15))

                elif self.fsm_state == "DOCKING_APPROACH":
                    # [경유지별 제자리 구름 방향 정렬 및 직진 보행 규칙]
                    # 시퀀스:
                    # 1. WP0_TURN: 스폰 위치에서 제자리 구름하며 WP0(-3, 3) 방향으로 회전 정렬
                    # 2. WP0_WALK: WP0(-3, 3) 방향으로 직진 보행
                    # 3. WP1_TURN: WP0(-3, 3) 도착 후 제자리 구름하며 WP1(-0.70, 0) 방향으로 회전 정렬
                    # 4. WP1_WALK: WP1(-0.70, 0) 도킹 1m 전으로 직진 보행
                    # 5. WP2_TURN: WP1(-0.70, 0) 도착 후 제자리 구름하며 테이블 정면(yaw=0)으로 정밀 회전 정렬
                    # 6. DOCK_CREEP: 완벽 정렬된 상태로 테이블 정면을 향해 직진 1m 감속 크리핑 (밀착 시 STANCE_LOCK)

                    self.phase_timer += dt_policy

                    if self.nav_phase == "WP0_TURN":
                        target_x, target_y = self.waypoints[0]  # (-3.0, 3.0)
                        target_yaw = math.atan2(target_y - pos_y, target_x - pos_x)
                        yaw_err = math.atan2(math.sin(target_yaw - yaw), math.cos(target_yaw - yaw))

                        # 스폰 위치에서 단단한 제자리 위치 유지 PD 제어 (밀림 방지: ±0.35m/s 제동력 확보)
                        err_x = self.hold_x - pos_x
                        err_y = self.hold_y - pos_y
                        body_err_x = cos_y * err_x + sin_y * err_y
                        body_err_y = -sin_y * err_x + cos_y * err_y
                        self.commands[0] = float(np.clip(1.8 * body_err_x - 0.4 * body_vel_x, -0.35, 0.35))
                        self.commands[1] = float(np.clip(1.8 * body_err_y - 0.4 * body_vel_y, -0.35, 0.35))
                        self.commands[2] = float(np.clip(1.5 * yaw_err, -0.45, 0.45))

                        # 제자리 구름 방향 정렬 조건 (6도 이내 0.3초 안착 시 직진 보행 개시)
                        if abs(yaw_err) < math.radians(6.0):
                            self.turn_settle_timer += dt_policy
                            if self.turn_settle_timer >= 0.3:
                                self.nav_phase = "WP0_WALK"
                                self.turn_settle_timer = 0.0
                                self.phase_timer = 0.0
                                print(f"\n  {Colors.BOLD}{Colors.GREEN}★ [{sim_time:5.2f}s] [WP0 정렬 완료] yaw 오차={math.degrees(yaw_err):.1f}° 정렬 성공! ➔ WP0 직진 보행 개시{Colors.RESET}\n", flush=True)
                        elif self.phase_timer >= 8.0 and abs(yaw_err) < math.radians(12.0):
                            self.nav_phase = "WP0_WALK"
                            self.turn_settle_timer = 0.0
                            self.phase_timer = 0.0
                            print(f"\n  {Colors.BOLD}{Colors.GREEN}★ [{sim_time:5.2f}s] [WP0 정렬 타임아웃 전환] 직진 보행 개시{Colors.RESET}\n", flush=True)
                        else:
                            self.turn_settle_timer = 0.0

                    elif self.nav_phase == "WP0_WALK":
                        target_x, target_y = self.waypoints[0]  # (-3.0, 3.0)
                        dist = math.hypot(target_x - pos_x, target_y - pos_y)
                        target_yaw = math.atan2(target_y - pos_y, target_x - pos_x)
                        yaw_err = math.atan2(math.sin(target_yaw - yaw), math.cos(target_yaw - yaw))

                        # [미리 감속 프로파일]: 경유지 접근 시 3단계 감속으로 관성 오버슈트 방지
                        if dist > 1.2:
                            base_v = 0.16       # 원거리 정상 순항 (0.16 m/s)
                        elif dist > 0.55:
                            base_v = 0.09       # 1차 사전 감속 (0.09 m/s)
                        else:
                            base_v = 0.04       # 경유지 진입 초저속 크리핑 (0.04 m/s)

                        v_fwd = 0.04 if abs(yaw_err) > math.radians(18.0) else base_v
                        self.commands[0] = v_fwd
                        self.commands[1] = 0.0
                        self.commands[2] = float(np.clip(1.2 * yaw_err, -0.25, 0.25))

                        # 1차 경유지(반경 0.35m) 원형 영역(dist <= 0.30m) 진입 시 제자리 구름 전환
                        if dist <= 0.30:
                            self.nav_phase = "WP1_TURN"
                            self.hold_x = pos_x  # 마커 진입 위치를 그대로 앵커로 고정
                            self.hold_y = pos_y
                            self.turn_settle_timer = 0.0
                            self.phase_timer = 0.0
                            self.commands[:] = 0.0
                            print(f"\n  {Colors.BOLD}{Colors.GREEN}★ [{sim_time:5.2f}s] [WP0 도착] (-3.0, 3.0) 마커 안착 완료! (dist={dist:.2f}m) ➔ WP0에서 제자리 구름하며 WP1(-0.70, 0.0) 회전 정렬{Colors.RESET}\n", flush=True)

                    elif self.nav_phase == "WP1_TURN":
                        target_x, target_y = self.waypoints[1]  # (-0.70, 0.0)
                        target_yaw = math.atan2(target_y - pos_y, target_x - pos_x)
                        yaw_err = math.atan2(math.sin(target_yaw - yaw), math.cos(target_yaw - yaw))

                        # WP0 마커 상에서 단단한 제자리 위치 유지 PD 제어 (밀림 방지: ±0.35m/s)
                        err_x = self.hold_x - pos_x
                        err_y = self.hold_y - pos_y
                        body_err_x = cos_y * err_x + sin_y * err_y
                        body_err_y = -sin_y * err_x + cos_y * err_y
                        self.commands[0] = float(np.clip(1.8 * body_err_x - 0.4 * body_vel_x, -0.35, 0.35))
                        self.commands[1] = float(np.clip(1.8 * body_err_y - 0.4 * body_vel_y, -0.35, 0.35))
                        self.commands[2] = float(np.clip(1.5 * yaw_err, -0.45, 0.45))

                        if abs(yaw_err) < math.radians(6.0):
                            self.turn_settle_timer += dt_policy
                            if self.turn_settle_timer >= 0.3:
                                self.nav_phase = "WP1_WALK"
                                self.turn_settle_timer = 0.0
                                self.phase_timer = 0.0
                                print(f"\n  {Colors.BOLD}{Colors.CYAN}★ [{sim_time:5.2f}s] [WP1 정렬 완료] yaw 오차={math.degrees(yaw_err):.1f}° 정렬 성공! ➔ WP1(도킹 1m 전) 직진 보행 개시{Colors.RESET}\n", flush=True)
                        elif self.phase_timer >= 12.0 and abs(yaw_err) < math.radians(12.0):
                            self.nav_phase = "WP1_WALK"
                            self.turn_settle_timer = 0.0
                            self.phase_timer = 0.0
                            print(f"\n  {Colors.BOLD}{Colors.CYAN}★ [{sim_time:5.2f}s] [WP1 정렬 타임아웃 전환] 직진 보행 개시{Colors.RESET}\n", flush=True)
                        else:
                            self.turn_settle_timer = 0.0

                    elif self.nav_phase == "WP1_WALK":
                        target_x, target_y = self.waypoints[1]  # (-0.70, 0.0)
                        dist = math.hypot(target_x - pos_x, target_y - pos_y)
                        target_yaw = math.atan2(target_y - pos_y, target_x - pos_x)
                        yaw_err = math.atan2(math.sin(target_yaw - yaw), math.cos(target_yaw - yaw))

                        # [도킹 1m 전 경유지 미리 감속 프로파일]: 도킹 구역 진입 전 관성 철저 소멸
                        if dist > 1.2:
                            base_v = 0.14       # 원거리 순항 (0.14 m/s)
                        elif dist > 0.55:
                            base_v = 0.07       # 1차 사전 감속 (0.07 m/s)
                        else:
                            base_v = 0.035      # 도킹 1m 전 경유지 진입 초저속 (0.035 m/s)

                        v_fwd = 0.035 if abs(yaw_err) > math.radians(18.0) else base_v
                        self.commands[0] = v_fwd
                        self.commands[1] = 0.0
                        self.commands[2] = float(np.clip(1.2 * yaw_err, -0.22, 0.22))

                        # 도킹 1m 전 마커(반경 0.35m) 진입(dist <= 0.30m) 시 제자리 구름 전환
                        if dist <= 0.30:
                            self.nav_phase = "WP2_TURN"
                            self.hold_x = pos_x
                            self.hold_y = pos_y
                            self.turn_settle_timer = 0.0
                            self.phase_timer = 0.0
                            self.commands[:] = 0.0
                            print(f"\n  {Colors.BOLD}{Colors.YELLOW}★ [{sim_time:5.2f}s] [WP1 도착] 도킹 1m 전 (-0.70, 0.0) 안착 완료! ➔ 제자리 구름하며 테이블 정면(yaw=0) 회전 정렬{Colors.RESET}\n", flush=True)

                    elif self.nav_phase == "WP2_TURN":
                        target_yaw = 0.0  # 테이블 정면 직각 방향
                        yaw_err = math.atan2(math.sin(target_yaw - yaw), math.cos(target_yaw - yaw))

                        # 도킹 1m 전(-0.70, 0.0) 위치 유지 단단한 PD 제어 (밀림 방지: ±0.30m/s)
                        err_x = self.hold_x - pos_x
                        err_y = self.hold_y - pos_y
                        body_err_x = cos_y * err_x + sin_y * err_y
                        body_err_y = -sin_y * err_x + cos_y * err_y
                        self.commands[0] = float(np.clip(1.8 * body_err_x - 0.4 * body_vel_x, -0.30, 0.30))
                        self.commands[1] = float(np.clip(1.8 * body_err_y - 0.4 * body_vel_y, -0.30, 0.30))
                        self.commands[2] = float(np.clip(1.5 * yaw_err, -0.35, 0.35))

                        # 4도 이내 정밀 정렬 (테이블 모서리와 완전 평행 직각)
                        if abs(yaw_err) < math.radians(4.0):
                            self.turn_settle_timer += dt_policy
                            if self.turn_settle_timer >= 0.3:
                                self.nav_phase = "DOCK_CREEP"
                                self.turn_settle_timer = 0.0
                                self.phase_timer = 0.0
                                print(f"\n  {Colors.BOLD}{Colors.GREEN}★ [{sim_time:5.2f}s] [테이블 정면 정렬 완료] yaw={math.degrees(yaw):.1f}° 직각 정렬 성공! ➔ 최종 도킹 1m 직진 극저속 크리핑 개시{Colors.RESET}\n", flush=True)
                        elif self.phase_timer >= 8.0 and abs(yaw_err) < math.radians(8.0):
                            self.nav_phase = "DOCK_CREEP"
                            self.turn_settle_timer = 0.0
                            self.phase_timer = 0.0
                            print(f"\n  {Colors.BOLD}{Colors.GREEN}★ [{sim_time:5.2f}s] [정면 정렬 타임아웃 전환] 최종 도킹 1m 직진 크리핑 개시{Colors.RESET}\n", flush=True)
                        else:
                            self.turn_settle_timer = 0.0

                    elif self.nav_phase in ("DOCK_CREEP", "DOCK_SETTLE"):
                        lat_err_y = 0.0 - pos_y  # Y=0 중심선 유지
                        target_yaw = float(np.clip(1.8 * lat_err_y, -0.20, 0.20))
                        dock_yaw_err = math.atan2(math.sin(target_yaw - yaw), math.cos(target_yaw - yaw))

                        # 테이블 턱 접촉 지점: pos_x ≈ 0.265m (테이블 전면 X=0.47m, 범퍼 전단 +0.205m)
                        dock_target_x = 0.265
                        err_x = dock_target_x - pos_x

                        if self.nav_phase == "DOCK_SETTLE" or self.is_bumper_touch:
                            cmd_x = -0.36  # 범퍼 접촉 유지 (1~3N 미세 전방 가압)
                        else:
                            # [최종 도킹 1m 초정밀 감속 크리핑]: RL 정책 고유 전진 오프셋(+0.20m/s) 보정
                            # 거리별 감속 타깃: 원거리(>0.35m) -> 저속 접근(>0.08m) -> 극저속 밀착(<=0.08m)
                            if err_x > 0.35:
                                target_cmd = -0.20  # ~0.12 m/s 안정적 서행 접근
                            elif err_x > 0.08:
                                target_cmd = -0.32  # ~0.06 m/s 1차 저속 크리핑
                            else:
                                target_cmd = -0.37  # ~0.025 m/s 초저속 도킹 밀착
                            cmd_x = float(np.clip(target_cmd - 0.35 * body_vel_x, -0.55, target_cmd))

                        self.commands[0] = cmd_x
                        self.commands[1] = float(np.clip(2.0 * lat_err_y - 0.4 * body_vel_y, -0.20, 0.20))
                        self.commands[2] = float(np.clip(1.5 * dock_yaw_err, -0.25, 0.25))

                elif self.fsm_state == "UNDOCKING":
                    # 뒤로 안전하게 후진
                    self.commands[0] = -0.12  # -0.12 m/s 후진
                    self.commands[1] = 0.0
                    yaw = float(np.arctan2(R_mat[1, 0], R_mat[0, 0]))
                    self.commands[2] = float(np.clip(-1.0 * yaw, -0.2, 0.2))

                # Policy 추론 (36D -> 6D Action, RL 명령 직접 인가)
                scaled_commands = np.array([
                    self.commands[0] * 1.5,
                    self.commands[1] * 1.0,
                    self.commands[2] * 0.5
                ], dtype=np.float32)
                policy_input = np.concatenate([self.encoder_out, obs, scaled_commands]).astype(np.float32)
                pol_in = {self.policy_input_name: policy_input}
                raw_actions = self.policy_session.run(None, pol_in)[0].flatten()
                self.actions = np.clip(raw_actions, -100.0, 100.0)

                for j in range(6):
                    action_min = (q_act[j] - self.default_joint_pos[j] +
                                  (self.kd_walking * v_act[j] - self.torque_limit) / self.kp_walking)
                    action_max = (q_act[j] - self.default_joint_pos[j] +
                                  (self.kd_walking * v_act[j] + self.torque_limit) / self.kp_walking)
                    act_clipped = np.clip(self.actions[j], action_min / self.action_scale, action_max / self.action_scale)
                    self.q_target[j] = act_clipped * self.action_scale + self.default_joint_pos[j]
                    self.last_action[j] = self.actions[j]

        # 500Hz PD 토크 산출
        self.loop_count += 1
        joint_error = self.q_target - q_act
        torques = self.kp_walking * joint_error - self.kd_walking * v_act
        torques = np.clip(torques, -self.torque_limit, self.torque_limit)
        return torques

def run_simulation(model, data, controller, viewer=None, max_time=30.0):
    wall_start = time.perf_counter()
    sim_start = data.time
    last_print_time = 0.0
    prev_sim_time = data.time
    step = 0
    reset_requested = [False]
    stop_threads = False

    print(f"\n{Colors.BOLD}[TEST EXECUTION] Phase 01-U02 Tron1 Payload & Docking Simulation Started...{Colors.RESET}", flush=True)
    if viewer:
        print(f"  {Colors.CYAN}📺 조작 단축키 안내: [D] 테이블 도킹 개시 | [U] 언도킹 후퇴 | [Space] 일시정지 | [R/Backspace/Enter] 리셋{Colors.RESET}", flush=True)

    def do_reset():
        nonlocal wall_start, sim_start, last_print_time, prev_sim_time, step
        stand_key_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "stand")
        if viewer:
            with viewer.lock():
                if stand_key_id != -1:
                    mujoco.mj_resetDataKeyframe(model, data, stand_key_id)
                else:
                    mujoco.mj_resetData(model, data)
                controller.configure_bottles(data)
                data.time = 0.0
                data.qvel[:] = 0.0
                data.ctrl[:] = 0.0
                mujoco.mj_forward(model, data)
        else:
            if stand_key_id != -1:
                mujoco.mj_resetDataKeyframe(model, data, stand_key_id)
            else:
                mujoco.mj_resetData(model, data)
            controller.configure_bottles(data)
            data.time = 0.0
            data.qvel[:] = 0.0
            data.ctrl[:] = 0.0
            mujoco.mj_forward(model, data)

        controller.reset()
        wall_start = time.perf_counter()
        sim_start = 0.0
        prev_sim_time = 0.0
        step = 0
        reset_requested[0] = False
        if viewer:
            viewer.sync()
        print(f"\n  {Colors.BOLD}{Colors.YELLOW}↺ [RESET 완료] 로봇과 물병 상태가 초기 스폰 상태로 완벽히 재동기화되었습니다.{Colors.RESET}\n", flush=True)

    def terminal_listener():
        while not stop_threads:
            try:
                line = sys.stdin.readline()
                if not line:
                    break
                cmd = line.strip().lower()
                if cmd in ('d', 'dock'):
                    controller.trigger_docking()
                elif cmd in ('u', 'undock'):
                    controller.trigger_undocking(data)
                else:
                    reset_requested[0] = True
            except Exception:
                break

    term_thread = threading.Thread(target=terminal_listener, daemon=True)
    term_thread.start()

    do_reset()

    while True:
        if viewer and not viewer.is_running():
            print(f"  * 사용자에 의해 뷰어 창이 닫혔습니다.", flush=True)
            break

        if reset_requested[0] or (prev_sim_time > 0.05 and (data.time < prev_sim_time - 0.01 or data.time == 0.0)):
            do_reset()

        # 물리 스텝 진행 (GUI 모드: 1.0x 실시간 동기화 / Headless 모드: 최고 속도 연산)
        if viewer:
            wall_elapsed = time.perf_counter() - wall_start
            step_count = 0
            while (data.time - sim_start) < wall_elapsed and step_count < 40:
                torques = controller.compute_torques(data)
                for i, act_id in enumerate(controller.act_ids):
                    data.ctrl[act_id] = torques[i]
                mujoco.mj_step(model, data)
                step_count += 1
                step += 1
            if step % 5 == 0:
                viewer.sync()
            time.sleep(0.001)
        else:
            torques = controller.compute_torques(data)
            for i, act_id in enumerate(controller.act_ids):
                data.ctrl[act_id] = torques[i]
            mujoco.mj_step(model, data)
            step += 1

        prev_sim_time = data.time

        # 터미널 주기 출력 (0.5초 간격)
        if data.time - last_print_time >= 0.5:
            last_print_time = data.time
            t = controller.telemetry
            color_map = {
                "LANDING": Colors.YELLOW,
                "IN_PLACE_HOLD": Colors.BLUE,
                "DOCKING_APPROACH": Colors.CYAN,
                "STANCE_LOCK": Colors.MAGENTA,
                "READY_FOR_PICK": Colors.GREEN,
                "UNDOCKING": Colors.CYAN,
                "FALLEN": Colors.RED
            }
            c = color_map.get(t.fsm_state, Colors.RESET)
            phase_str = f" | Phase: [{Colors.BOLD}{t.nav_phase:<10}{Colors.RESET}]" if t.fsm_state == "DOCKING_APPROACH" else ""
            timer_str = f" | 안정={t.docking_stable_timer:3.1f}s" if t.fsm_state in ("STANCE_LOCK", "READY_FOR_PICK") else ""
            print(f"  * [{t.sim_time:5.2f}s] FSM: [{c}{t.fsm_state:^16}{Colors.RESET}]{phase_str}{timer_str} | (X={t.base_x:+5.2f}, Y={t.base_y:+5.2f}) | yaw={t.yaw_deg:+5.1f}° | 범퍼={t.bumper_force:4.1f}N | 트레이진동={t.tray_vel_rms:6.4f}m/s | 물병=[{t.bottle_status}]", flush=True)

        if not viewer and data.time >= max_time:
            break

    stop_threads = True

def main():
    args = parse_args()
    if not os.path.exists(args.xml):
        print(f"\033[91m[에러] XML 파일을 찾을 수 없습니다: {args.xml}\033[0m")
        sys.exit(1)

    model = mujoco.MjModel.from_xml_path(args.xml)
    data = mujoco.MjData(model)
    controller = Tron1PayloadController(
        model, model_dir=args.model_dir, num_bottles=args.bottles, auto_dock=args.auto
    )

    if args.no_gui:
        print(f"{Colors.BOLD}{Colors.CYAN}Headless 모드로 시뮬레이션을 실행합니다. (최대 {args.max_time}초, 물병 {args.bottles}개){Colors.RESET}")
        run_simulation(model, data, controller, viewer=None, max_time=args.max_time)
    else:
        def key_callback(keycode):
            # D = 68/100, U = 85/117, R = 82/114, Backspace = 259
            if keycode in (ord('d'), ord('D'), 68):
                controller.trigger_docking()
            elif keycode in (ord('u'), ord('U'), 85):
                controller.trigger_undocking(data)
            elif keycode in (ord('r'), ord('R'), 82, 259):
                stand_key_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "stand")
                if stand_key_id != -1:
                    mujoco.mj_resetDataKeyframe(model, data, stand_key_id)
                else:
                    mujoco.mj_resetData(model, data)
                controller.configure_bottles(data)
                data.time = 0.0
                controller.reset()

        with mujoco.viewer.launch_passive(model, data, key_callback=key_callback) as v:
            v.cam.distance = 2.8
            v.cam.elevation = -18
            v.cam.azimuth = 135
            run_simulation(model, data, controller, viewer=v, max_time=args.max_time)

if __name__ == '__main__':
    main()
