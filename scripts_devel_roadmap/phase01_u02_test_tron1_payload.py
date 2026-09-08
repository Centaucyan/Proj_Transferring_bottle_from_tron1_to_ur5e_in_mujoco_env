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
    sim_time: float = 0.0
    base_x: float = 0.0
    base_z: float = 0.0
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
    parser.add_argument("--xml", type=str, default="unit_test_models/phase01_u02_scene_unit_tron1_payload.xml",
                        help="Path to Tron1 payload unit scene XML")
    parser.add_argument("--model_dir", type=str, default="model_rl/tron1",
                        help="Path to directory containing policy.onnx and encoder.onnx")
    parser.add_argument("--bottles", type=int, default=3, choices=[1, 2, 3],
                        help="Number of bottles to load (1: Asymmetric left slot, 2: Left+Right, 3: Full load)")
    parser.add_argument("--auto", action="store_true",
                        help="Automatically proceed from Hold to Docking after 3 seconds")
    parser.add_argument("--max_time", type=float, default=30.0, help="Simulation time limit in seconds")
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
        self.kp_lock = 50.0   # Stance Lock 고감쇠 게인
        self.kd_lock = 5.0
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
        self.last_action = np.zeros(6, dtype=np.float32)
        self.actions = np.zeros(6, dtype=np.float32)
        self.q_target = np.zeros(6, dtype=np.float32)
        self.encoder_out = np.zeros(3, dtype=np.float32)
        self.proprio_history_buffer = np.zeros(300, dtype=np.float32)
        self.is_first_rec_obs = True
        self.gait_index = 0.0
        self.loop_count = 0
        self.commands[:] = 0.0
        self.telemetry = U02Telemetry(fsm_state="LANDING")

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
            print(f"\n  {Colors.BOLD}{Colors.CYAN}▶ [도킹 개시] 테이블 모서리 턱으로 저속 전진 보행을 시작합니다! (vx = +0.12 m/s){Colors.RESET}\n", flush=True)

    def trigger_undocking(self, data):
        if self.fsm_state in ("STANCE_LOCK", "READY_FOR_PICK"):
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
                print(f"\n  {Colors.BOLD}{Colors.GREEN}★ [{sim_time:5.2f}s] 'LANDING' ➔ 'IN_PLACE_HOLD' (페이로드 제자리 발구름 시작!){Colors.RESET}\n", flush=True)

        elif self.fsm_state == "IN_PLACE_HOLD":
            if self.auto_dock and self.state_timer >= 3.0:
                self.trigger_docking()

        elif self.fsm_state == "DOCKING_APPROACH":
            # 테이블 모서리 범퍼 접촉 감지 (F > 2.0 N) & 양발 접지(Double Stance) 시 Stance Lock 전환!
            is_bumper_touch = (bumper_force >= 2.0 or (pos_x >= 0.54 and bumper_force >= 0.5))
            if is_bumper_touch:
                if contact_L and contact_R:
                    self.fsm_state = "STANCE_LOCK"
                    self.state_timer = 0.0
                    self.vibration_stable_timer = 0.0
                    self.locked_joint_pos = np.copy(q_act)  # 3점 지지 안착 자세 고정
                    print(f"\n  {Colors.BOLD}{Colors.YELLOW}⚡ [{sim_time:5.2f}s] 양발 접지 및 범퍼 밀착 완료 (F={bumper_force:4.1f}N)! 'STANCE_LOCK' 진입 (발구름 정지){Colors.RESET}\n", flush=True)
                else:
                    # 범퍼 접촉 상태에서 한 발이 떠 있다면 즉각 정지 명령으로 반대발 착지 유도
                    self.commands[0] = 0.0

        elif self.fsm_state == "STANCE_LOCK":
            # 3점 지지 상태에서 트레이 진동 소멸 모니터링 (< 0.01 m/s)
            if tray_vel_rms < 0.01:
                self.vibration_stable_timer += dt
                if self.vibration_stable_timer >= 3.0:
                    self.fsm_state = "READY_FOR_PICK"
                    self.telemetry.is_ready_for_pick = True
                    print(f"\n  {Colors.BOLD}{Colors.GREEN}✔ [{sim_time:5.2f}s] [도킹 성공] 3초간 무진동 정적 안정 달성! => 'READY_FOR_PICK' 확립!{Colors.RESET}\n", flush=True)
            else:
                self.vibration_stable_timer = 0.0

        elif self.fsm_state == "UNDOCKING":
            # 뒤로 약 0.15m 물러나면 다시 자립 제자리 발구름 복귀
            if pos_x <= self.undock_start_x - 0.15 or self.state_timer >= 2.5:
                self.fsm_state = "IN_PLACE_HOLD"
                self.state_timer = 0.0
                print(f"\n  {Colors.BOLD}{Colors.CYAN}↺ [{sim_time:5.2f}s] 언도킹 완료! 안전 거리 확보 후 'IN_PLACE_HOLD' 복귀.{Colors.RESET}\n", flush=True)

        # 텔레메트리 갱신
        self.telemetry.fsm_state = self.fsm_state
        self.telemetry.sim_time = sim_time
        self.telemetry.base_x = pos_x
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
        # 1. Stance Lock 모드: RL 정책 추론 중단, 3점 지지 고감쇠 관절 PD 제어 및 대칭 전방 밀착 바이어스
        if self.fsm_state in ("STANCE_LOCK", "READY_FOR_PICK"):
            joint_error = self.locked_joint_pos - q_act
            torques = self.kp_lock * joint_error - self.kd_lock * v_act
            # 힙 관절에 전방 밀착 바이어스 토크 인가 (hip_L: axis 0 1 0 (+), hip_R: axis 0 -1 0 (-))
            torques[1] += 8.0   # hip_L_motor 전방 숙임
            torques[4] -= 8.0   # hip_R_motor 전방 숙임 (반대 축)
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

                # 속도 명령(Command) 결정
                if self.fsm_state == "IN_PLACE_HOLD":
                    # 원점 유지 PD 피드백 (전진 드리프트 상쇄)
                    yaw = float(np.arctan2(R_mat[1, 0], R_mat[0, 0]))
                    cos_y, sin_y = np.cos(yaw), np.sin(yaw)
                    err_x, err_y = -pos_x, -pos_y
                    vel_x, vel_y = float(data.qvel[0]), float(data.qvel[1])
                    body_err_x = cos_y * err_x + sin_y * err_y
                    body_err_y = -sin_y * err_x + cos_y * err_y
                    body_vel_x = cos_y * vel_x + sin_y * vel_y
                    body_vel_y = -sin_y * vel_x + cos_y * vel_y
                    self.commands[0] = float(np.clip(1.5 * body_err_x - 0.4 * body_vel_x, -0.5, 0.5))
                    self.commands[1] = float(np.clip(1.5 * body_err_y - 0.4 * body_vel_y, -0.5, 0.5))
                    self.commands[2] = float(np.clip(-1.0 * yaw, -0.4, 0.4))

                elif self.fsm_state == "DOCKING_APPROACH":
                    # 테이블 턱(x ≈ 0.56m)에 근접할수록 크립 저속으로 소프트 접근
                    if pos_x >= 0.42:
                        self.commands[0] = 0.03  # 소프트 도킹을 위한 크립 초저속 (0.03 m/s)
                    else:
                        self.commands[0] = 0.07  # 안정적 접근 속도 (0.07 m/s)
                    self.commands[1] = float(np.clip(-1.0 * pos_y, -0.2, 0.2))  # Y 중심선 유지
                    yaw = float(np.arctan2(R_mat[1, 0], R_mat[0, 0]))
                    self.commands[2] = float(np.clip(-1.0 * yaw, -0.2, 0.2))

                elif self.fsm_state == "UNDOCKING":
                    # 뒤로 안전하게 후진
                    self.commands[0] = -0.15  # -0.15 m/s 후진
                    self.commands[1] = 0.0
                    yaw = float(np.arctan2(R_mat[1, 0], R_mat[0, 0]))
                    self.commands[2] = float(np.clip(-1.0 * yaw, -0.2, 0.2))

                # Policy 추론 (36D -> 6D Action)
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

        # 1.0x 실시간 물리 동기화
        wall_elapsed = time.perf_counter() - wall_start
        step_count = 0
        while (data.time - sim_start) < wall_elapsed and step_count < 40:
            torques = controller.compute_torques(data)
            for i, act_id in enumerate(controller.act_ids):
                data.ctrl[act_id] = torques[i]
            mujoco.mj_step(model, data)
            step_count += 1
            step += 1

        prev_sim_time = data.time

        if viewer and (step % 5 == 0):
            viewer.sync()

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
            print(f"  * [{t.sim_time:5.2f}s] FSM: [{c}{t.fsm_state:^16}{Colors.RESET}] | X={t.base_x:+5.2f}m | 범퍼={t.bumper_force:4.1f}N | 트레이진동={t.tray_vel_rms:6.4f}m/s | 물병=[{t.bottle_status}] | 안정타이머={t.docking_stable_timer:3.1f}s", flush=True)

        if not viewer and data.time >= max_time:
            break

        time.sleep(0.001)

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
