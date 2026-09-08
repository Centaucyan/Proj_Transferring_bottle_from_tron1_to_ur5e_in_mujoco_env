#!/usr/bin/env python3
"""
scripts_devel_roadmap/phase01_u01_test_tron1_walking_rl.py
Phase 01-U01: Tron1 LimX Official Pretrained RL In-place Stepping & Balancing
- Loads official pretrained ONNX models (policy.onnx, encoder.onnx) from model_rl/tron1/
- 500Hz Policy Inference with Projected Gravity & Proprioceptive Observations
- High-frequency Joint PD torque execution (Kp=42.0, Kd=3.5)
- In-place Stepping Origin Hold Feedback (PD compensation on vx, vy, wz commands)
- 1.0x Real-time Physics Speed Synchronization
- Interactive Viewer Auto-Reset Support (Instant sync on Reset button / Backspace / R / Terminal Enter)
- Telemetry monitoring: Base height Z, gyro rates, pitch, and in-place stepping stability
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

@dataclass
class RobotState:
    state: str = "landing"         # 'landing' -> 'stepping' (전도 시 'falling')
    sim_time: float = 0.0
    pos_x: float = 0.0
    pos_z: float = 0.0
    pitch: float = 0.0
    pitch_deg: float = 0.0
    gyro_norm: float = 0.0
    touch_L: bool = False
    touch_R: bool = False
    is_fallen: bool = False

class Colors:
    GREEN = '\033[92m'
    RED = '\033[91m'
    YELLOW = '\033[93m'
    CYAN = '\033[96m'
    BOLD = '\033[1m'
    RESET = '\033[0m'

def parse_args():
    parser = argparse.ArgumentParser(description="Phase 01-U01: Tron1 Pretrained RL In-place Stepping")
    parser.add_argument("--xml", type=str, default="xml_for_unit_test/phase01_u01_scene_unit_tron1.xml",
                        help="Path to Tron1 unit scene XML")
    parser.add_argument("--model_dir", type=str, default="model_rl/tron1",
                        help="Path to directory containing policy.onnx and encoder.onnx")
    parser.add_argument("--max_time", type=float, default=20.0, help="Maximum simulation time limit in seconds")
    parser.add_argument("--no-gui", action="store_true", help="Run simulation in headless mode without 3D viewer")
    parser.add_argument("--no-hold", action="store_true", help="Disable origin position hold feedback")
    return parser.parse_args()

class Tron1RLController:
    """
    LimX Dynamics 공식 상용 사전훈련 ONNX 모델 기반 제자리 발구름(In-place Stepping) 제어기
    """
    def __init__(self, model, model_dir="model_rl/tron1", hold_position=True):
        self.model = model
        self.model_dir = model_dir
        self.hold_position = hold_position

        self.actuator_names = [
            "abad_L_motor", "hip_L_motor", "knee_L_motor",
            "abad_R_motor", "hip_R_motor", "knee_R_motor"
        ]
        self.act_ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name) for name in self.actuator_names]
        self.base_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_Link")
        self.foot_L_geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "foot_L_col")
        self.foot_R_geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "foot_R_col")

        # LimX PF_TRON1A 공식 제어 매개변수 (params.yaml 정합)
        self.default_joint_pos = np.zeros(6, dtype=np.float32)
        self.kp = 42.0
        self.kd = 3.5
        self.action_scale = 0.25
        self.torque_limit = 80.0
        self.decimation = 10  # 500Hz / 10 = 50Hz RL Policy Loop

        # 관측 정규화 및 크기 설정
        self.observations_size = 30
        self.obs_history_length = 10
        self.gait = np.array([2.0, 0.5, 0.5, 0.1], dtype=np.float32)
        self.commands = np.zeros(3, dtype=np.float32)  # 속도 명령: [vx, vy, wz]

        # ONNX 세션 초기화
        self.policy_path = os.path.join(model_dir, "policy.onnx")
        self.encoder_path = os.path.join(model_dir, "encoder.onnx")

        self.has_onnx = os.path.exists(self.policy_path) and os.path.exists(self.encoder_path)
        if not self.has_onnx:
            print(f"\n{Colors.BOLD}{Colors.YELLOW}[주의] ONNX 모델 파일을 찾을 수 없습니다: '{self.model_dir}'{Colors.RESET}")
            print(f"  * 다운로드 명령:")
            print(f"    wget -O {self.policy_path} https://github.com/limxdynamics/tron1-rl-deploy-python/raw/main/controllers/model/PF_TRON1A/policy/isaacgym/policy.onnx")
            print(f"    wget -O {self.encoder_path} https://github.com/limxdynamics/tron1-rl-deploy-python/raw/main/controllers/model/PF_TRON1A/policy/isaacgym/encoder.onnx\n")
            self.policy_session = None
            self.encoder_session = None
        else:
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
            print(f"  * Policy Input : name='{self.policy_input_name}', shape={self.policy_session.get_inputs()[0].shape}")
            print(f"  * Encoder Input: name='{self.encoder_input_name}', shape={self.encoder_session.get_inputs()[0].shape}")

        self.robot_state = RobotState(state="landing")
        self.reset()

    def reset(self):
        self.robot_state = RobotState(state="landing")
        self.last_action = np.zeros(6, dtype=np.float32)
        self.actions = np.zeros(6, dtype=np.float32)
        self.q_target = np.zeros(6, dtype=np.float32)
        self.encoder_out = np.zeros(3, dtype=np.float32)
        self.proprio_history_buffer = np.zeros(300, dtype=np.float32)
        self.is_first_rec_obs = True
        self.gait_index = 0.0
        self.loop_count = 0
        self.landing_timer = 0.0

    def compute_torques(self, data):
        sim_time = data.time
        pos_z = data.xpos[self.base_body_id][2]
        q_act = data.qpos[7:13].astype(np.float32)
        v_act = data.qvel[6:12].astype(np.float32)

        # 1. IMU 센서 데이터 추출
        quat = data.sensor("imu_quat").data  # [w, x, y, z]
        gyro = data.sensor("imu_gyro").data  # [wx, wy, wz]
        w, x, y, z = quat
        sinp = 2.0 * (w * y - z * x)
        pitch = math.asin(np.clip(sinp, -1.0, 1.0))
        pitch_deg = math.degrees(pitch)
        gyro_norm = np.linalg.norm(gyro)

        # 2. 접촉 여부 검출
        contact_L = False
        contact_R = False
        for i in range(data.ncon):
            con = data.contact[i]
            if con.geom1 == self.foot_L_geom_id or con.geom2 == self.foot_L_geom_id:
                contact_L = True
            if con.geom1 == self.foot_R_geom_id or con.geom2 == self.foot_R_geom_id:
                contact_R = True

        self.robot_state.sim_time = sim_time
        self.robot_state.pos_x = data.xpos[self.base_body_id][0]
        self.robot_state.pos_z = pos_z
        self.robot_state.pitch = pitch
        self.robot_state.pitch_deg = pitch_deg
        self.robot_state.gyro_norm = gyro_norm
        self.robot_state.touch_L = contact_L
        self.robot_state.touch_R = contact_R

        # 3. 전도 감지 (Z < 0.40m 또는 45도 이상 기울어짐)
        if sim_time > 0.15 and (pos_z < 0.40 or abs(pitch_deg) > 45.0):
            if self.robot_state.state != "falling":
                self.robot_state.state = "falling"
                self.robot_state.is_fallen = True
                print(f"\n  {Colors.BOLD}{Colors.RED}✗ [{sim_time:5.2f}s] 전도 감지 (Z={pos_z:.2f}m, Pitch={pitch_deg:.1f}°){Colors.RESET}\n", flush=True)

        # 4. FSM 상태 전이 (landing 0.15초 후 즉각 stepping 진입)
        if self.robot_state.state == "landing":
            self.landing_timer += self.model.opt.timestep
            if self.landing_timer >= 0.15 and (contact_L or contact_R or self.landing_timer >= 0.25):
                self.robot_state.state = "stepping"
                print(f"\n  {Colors.BOLD}{Colors.CYAN}★ [{sim_time:5.2f}s] [상태 전이] 'landing' ➔ 'stepping' (LimX RL 발구름 개시!){Colors.RESET}\n", flush=True)

        # 5. RL 정책 추론 (Decimation: 10스텝마다 1회 = 50Hz)
        if self.has_onnx and self.robot_state.state != "falling":
            if self.loop_count % self.decimation == 0:
                # 5-1. 투영 중력 벡터 계산: R(q)^T * [0, 0, -1]
                R_mat = np.zeros(9)
                mujoco.mju_quat2Mat(R_mat, quat)
                R_mat = R_mat.reshape(3, 3)
                proj_gravity = (R_mat.T @ np.array([0.0, 0.0, -1.0], dtype=np.float32)).astype(np.float32)

                # 5-2. LimX 공식 30차원 Observation 벡터 구성
                base_ang_vel = (gyro * 0.25).astype(np.float32)
                joint_pos_input = ((q_act - self.default_joint_pos) * 1.0).astype(np.float32)
                joint_velocities = (v_act * 0.05).astype(np.float32)
                actions_prev = self.last_action.astype(np.float32)

                # Gait Clock 계산
                self.gait_index += 0.02 * self.gait[0]
                if self.gait_index > 1.0:
                    self.gait_index = 0.0
                gait_clock = np.array([
                    np.sin(self.gait_index * 2.0 * np.pi),
                    np.cos(self.gait_index * 2.0 * np.pi)
                ], dtype=np.float32)

                # 30차원 결합: [ang_vel(3), proj_g(3), q_pos(6), q_vel(6), action(6), gait_clock(2), gait(4)]
                obs = np.concatenate([
                    base_ang_vel, proj_gravity, joint_pos_input,
                    joint_velocities, actions_prev, gait_clock, self.gait
                ]).astype(np.float32)
                obs = np.clip(obs, -100.0, 100.0)

                # 5-3. 히스토리 버퍼 갱신 (10스텝 x 30차원 = 300차원 1D 텐서)
                if self.is_first_rec_obs:
                    for i in range(self.obs_history_length):
                        self.proprio_history_buffer[i * self.observations_size:(i + 1) * self.observations_size] = obs
                    self.is_first_rec_obs = False
                else:
                    self.proprio_history_buffer[:-self.observations_size] = self.proprio_history_buffer[self.observations_size:]
                    self.proprio_history_buffer[-self.observations_size:] = obs

                # 5-4. Encoder 순전파 (300차원 1D -> 3차원 잠재 벡터)
                enc_in = {self.encoder_input_name: self.proprio_history_buffer}
                self.encoder_out = self.encoder_session.run(None, enc_in)[0].flatten()

                # 5-5. 제자리 위치 유지 (Origin Position Hold Feedback)
                # 로봇이 스폰 원점 (0, 0)에서 벗어나면 위치/속도 오차를 바탕으로 반대 방향 속도 명령 자동 인가
                if self.hold_position:
                    pos_x = float(data.xpos[self.base_body_id][0])
                    pos_y = float(data.xpos[self.base_body_id][1])
                    vel_x = float(data.qvel[0])
                    vel_y = float(data.qvel[1])

                    # 로봇 Yaw 각도를 고려하여 바디 로컬 오차로 변환
                    yaw = float(np.arctan2(R_mat[1, 0], R_mat[0, 0]))
                    cos_y, sin_y = np.cos(yaw), np.sin(yaw)
                    err_world_x = -pos_x  # 목표 위치: x = 0.0
                    err_world_y = -pos_y  # 목표 위치: y = 0.0

                    body_err_x = cos_y * err_world_x + sin_y * err_world_y
                    body_err_y = -sin_y * err_world_x + cos_y * err_world_y
                    body_vel_x = cos_y * vel_x + sin_y * vel_y
                    body_vel_y = -sin_y * vel_x + cos_y * vel_y

                    # 비례-미분(PD) 피드백 속도 명령 생성 (전진 드리프트 완벽 상쇄)
                    self.commands[0] = float(np.clip(1.5 * body_err_x - 0.4 * body_vel_x, -0.6, 0.6))
                    self.commands[1] = float(np.clip(1.5 * body_err_y - 0.4 * body_vel_y, -0.6, 0.6))
                    self.commands[2] = float(np.clip(-1.0 * yaw, -0.4, 0.4))

                # 5-6. Policy 순전파 (36차원 1D = latent 3 + obs 30 + cmd 3 -> 6차원 액션)
                scaled_commands = np.array([
                    self.commands[0] * 1.5,
                    self.commands[1] * 1.0,
                    self.commands[2] * 0.5
                ], dtype=np.float32)
                policy_input = np.concatenate([self.encoder_out, obs, scaled_commands]).astype(np.float32)
                pol_in = {self.policy_input_name: policy_input}
                raw_actions = self.policy_session.run(None, pol_in)[0].flatten()
                self.actions = np.clip(raw_actions, -100.0, 100.0)

                # 5-7. 토크 한계 기반 목표 관절 각도 변환 (LimX 공식 클리핑 로직)
                for j in range(6):
                    action_min = (q_act[j] - self.default_joint_pos[j] +
                                  (self.kd * v_act[j] - self.torque_limit) / self.kp)
                    action_max = (q_act[j] - self.default_joint_pos[j] +
                                  (self.kd * v_act[j] + self.torque_limit) / self.kp)
                    act_clipped = np.clip(self.actions[j], action_min / self.action_scale, action_max / self.action_scale)
                    self.q_target[j] = act_clipped * self.action_scale + self.default_joint_pos[j]
                    self.last_action[j] = self.actions[j]

        # 6. 관절 토크 연산 (500Hz 고주파 PD 제어)
        self.loop_count += 1
        joint_error = self.q_target - q_act
        torques = self.kp * joint_error - self.kd * v_act
        torques = np.clip(torques, -self.torque_limit, self.torque_limit)
        return torques

def run_simulation(model, data, controller, viewer=None, max_time=20.0):
    wall_start = time.perf_counter()
    sim_start = data.time
    last_print_time = 0.0
    prev_sim_time = data.time
    step = 0
    reset_requested = [False]
    stop_threads = False

    print(f"\n{Colors.BOLD}[TEST EXECUTION] Running Tron1 RL In-place Stepping Simulation...{Colors.RESET}", flush=True)
    if viewer:
        print(f"  {Colors.CYAN}📺 3D MuJoCo 뷰어가 활성화되었습니다. (Space: 일시정지, Backspace / R / 터미널 Enter: 리셋){Colors.RESET}", flush=True)

    def do_reset():
        nonlocal wall_start, sim_start, last_print_time, prev_sim_time, step
        stand_key_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "stand")
        if viewer:
            with viewer.lock():
                if stand_key_id != -1:
                    mujoco.mj_resetDataKeyframe(model, data, stand_key_id)
                else:
                    mujoco.mj_resetData(model, data)
                data.time = 0.0
                data.qvel[:] = 0.0
                data.ctrl[:] = 0.0
                mujoco.mj_forward(model, data)
        else:
            if stand_key_id != -1:
                mujoco.mj_resetDataKeyframe(model, data, stand_key_id)
            else:
                mujoco.mj_resetData(model, data)
            data.time = 0.0
            data.qvel[:] = 0.0
            data.ctrl[:] = 0.0
            mujoco.mj_forward(model, data)

        controller.reset()
        controller.robot_state.pos_z = float(data.xpos[controller.base_body_id][2])
        controller.robot_state.pos_x = float(data.xpos[controller.base_body_id][0])
        wall_start = time.perf_counter()
        sim_start = 0.0
        prev_sim_time = 0.0
        step = 0
        reset_requested[0] = False
        if viewer:
            viewer.sync()
        rs = controller.robot_state
        print(f"\n  {Colors.BOLD}{Colors.YELLOW}↺ [RESET 완료] 시뮬레이션 및 로봇 상태가 초기 스폰 상태(robot_state='landing')로 완벽히 재동기화되었습니다.{Colors.RESET}", flush=True)
        print(f"  * [ 0.00s] robot_state: [{rs.state:^11}] | Pitch={rs.pitch_deg:+5.1f}° | 높이 Z={rs.pos_z:5.3f}m | Gyro={rs.gyro_norm:5.2f} rad/s\n", flush=True)

    def terminal_listener():
        while not stop_threads:
            try:
                line = sys.stdin.readline()
                if not line:
                    break
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

        # Reset 감지 (뷰어 UI Reset 버튼 또는 터미널/단축키)
        if reset_requested[0] or (prev_sim_time > 0.05 and (data.time < prev_sim_time - 0.01 or data.time == 0.0)):
            do_reset()

        # 1.0x 완벽 실시간 물리 동기화
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

        # 뷰어 화면 동기화
        if viewer and (step % 5 == 0):
            viewer.sync()

        # 터미널 텔레메트리 주기 출력 (0.5초 간격)
        if data.time - last_print_time >= 0.5:
            last_print_time = data.time
            rs = controller.robot_state
            touch_str = f"L:{'ON ' if rs.touch_L else 'OFF'} R:{'ON ' if rs.touch_R else 'OFF'}"
            status_color = Colors.GREEN if rs.state == "stepping" else (Colors.RED if rs.state == "falling" else Colors.YELLOW)
            print(f"  * [{data.time:5.2f}s] robot_state: [{status_color}{rs.state:^11}{Colors.RESET}] | X={rs.pos_x:+5.2f}m | 높이 Z={rs.pos_z:5.3f}m | cmd_vx={controller.commands[0]:+5.2f}m/s | 발접촉=[{touch_str}] | Gyro={rs.gyro_norm:5.2f} rad/s", flush=True)

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
    controller = Tron1RLController(model, model_dir=args.model_dir, hold_position=not args.no_hold)

    if args.no_gui:
        print(f"{Colors.BOLD}{Colors.CYAN}Headless 모드로 시뮬레이션을 실행합니다. (최대 {args.max_time}초){Colors.RESET}")
        run_simulation(model, data, controller, viewer=None, max_time=args.max_time)
    else:
        # 키보드 이벤트 핸들러
        def key_callback(keycode):
            # GLFW keycodes: R=82, Backspace=259
            if keycode in (ord('r'), ord('R'), 82, 259):
                stand_key_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "stand")
                if stand_key_id != -1:
                    mujoco.mj_resetDataKeyframe(model, data, stand_key_id)
                else:
                    mujoco.mj_resetData(model, data)
                data.time = 0.0
                controller.reset()

        with mujoco.viewer.launch_passive(model, data, key_callback=key_callback) as v:
            v.cam.distance = 2.4
            v.cam.elevation = -15
            v.cam.azimuth = 135
            run_simulation(model, data, controller, viewer=v, max_time=args.max_time)

if __name__ == '__main__':
    main()
