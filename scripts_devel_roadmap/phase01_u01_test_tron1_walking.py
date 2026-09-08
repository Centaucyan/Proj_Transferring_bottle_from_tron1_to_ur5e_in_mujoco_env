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
import numpy as np
import mujoco
import mujoco.viewer

class Colors:
    GREEN = '\033[92m'
    RED = '\033[91m'
    YELLOW = '\033[93m'
    CYAN = '\033[96m'
    BOLD = '\033[1m'
    RESET = '\033[0m'

def parse_args():
    parser = argparse.ArgumentParser(description="Phase 01-U01: Tron1 Spawn Pose Hold Test")
    parser.add_argument("--headless", action="store_true", help="Run in headless text-only mode (no GUI window)")
    parser.add_argument("--xml", type=str, default="unit_test_models/phase01_u01_scene_unit_tron1.xml",
                        help="Path to Tron1 unit scene XML")
    parser.add_argument("--max_time", type=float, default=10.0, help="Maximum simulation time limit in seconds (headless mode)")
    return parser.parse_args()

class Tron1PoseHoldController:
    """
    Tron1 스폰 자세 유지(Pose Hold) 전용 관절 PD 제어기
    - XML 'stand' 키프레임 관절 각도를 목표값(q_des)으로 자동 로드
    - 목표 관절 각도를 단단하게 유지하도록 고강성 PD 제어기 동작
    - 전도(추락) 및 관절 추종 오차 모니터링
    """
    def __init__(self, model):
        self.model = model

        self.actuator_names = [
            "abad_L_motor", "hip_L_motor", "knee_L_motor",
            "abad_R_motor", "hip_R_motor", "knee_R_motor"
        ]
        
        self.act_ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name) for name in self.actuator_names]
        self.base_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_Link")

        # 기립 기본 관절 각도 (XML의 'stand' 키프레임에서 자동 로드, 없으면 기본값 사용)
        stand_key_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "stand")
        if stand_key_id != -1:
            self.q_target = model.key_qpos[stand_key_id][7:13].copy()
        else:
            self.q_target = np.array([0.0, 0.3, 1.25, 0.0, -0.3, -1.25])

        # 스폰 자세 유지용 관절 PD 게인
        self.kp = np.array([200.0, 250.0, 250.0,  200.0, 250.0, 250.0])
        self.kd = np.array([12.0, 15.0, 15.0,     12.0, 15.0, 15.0])

        # IMU 상체 수평(Pitch) 능동 제어 게인
        self.kp_pitch = 1.2   # 비례 게인 (P)
        self.ki_pitch = 2.5   # [추가] 적분 게인 (I) - 잔류 편차를 0°로 완전히 제거!
        self.kd_pitch = 0.08  # 미분 게인 (D) - 흔들림 완충
        self.pitch_integral = 0.0

        # [신규] 수평 유지 및 전도 감지 판정 파라미터
        self.pitch_tolerance = math.radians(3.0)  # 수평 판정 허용 각도 (±3.0°, 약 0.052 rad)
        self.tilt_max_time = 3.0                 # 수평 불량 지속 허용 한계 시간 (초)
        self.tilt_duration = 0.0                 # 수평을 벗어난 누적 지속 시간 (초)
        self.current_pitch = 0.0                 # 현재 IMU 피치 각도 (rad)

        self.reset()

    def reset(self):
        self.state = "HOLD_SPAWN"
        self.pitch_integral = 0.0
        self.tilt_duration = 0.0
        self.current_pitch = 0.0
        self.log_time = []
        self.log_z = []
        self.log_pitch = []
        self.log_gyro = []
        self.log_max_error = []
        self.log_fall = False

    def compute_torques(self, data):
        sim_time = data.time
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
        pitch_rate = gyro[1]                          # Pitch축 회전 각속도
        self.current_pitch = pitch

        # =========================================================================
        # 수평 불량 및 전도 감지 (IMU Pitch 수평 벗어남이 3초 이상 지속될 때)
        # =========================================================================
        dt = self.model.opt.timestep
        if abs(pitch) > self.pitch_tolerance:
            self.tilt_duration += dt
            if self.tilt_duration >= self.tilt_max_time:
                self.log_fall = True
        else:
            # 수평(±3도 이내) 정상 범위 복귀 시 타이머 리셋
            self.tilt_duration = 0.0

        # 2. 오차 적분 누적 (물리 스텝 dt 주기 적분, 와인드업 방지 클리핑)
        self.pitch_integral += pitch * dt
        self.pitch_integral = np.clip(self.pitch_integral, -0.6, 0.6)

        # =========================================================================
        # 3. 수평(Pitch = 0) 유지를 위한 PID 보정 각도 계산 (P + I + D)
        # =========================================================================
        pitch_correction = (self.kp_pitch * pitch + 
                            self.ki_pitch * self.pitch_integral + 
                            self.kd_pitch * pitch_rate)

        # =========================================================================
        # 4. 목표 관절 각도에 피치 보정치 반영 (고관절을 조절하여 상체 수평 복원)
        # =========================================================================
        q_des = self.q_target.copy()
        q_des[1] += pitch_correction  # Left hip: 각도를 줄여 상체를 앞으로 숙임
        q_des[4] -= pitch_correction  # Right hip: 각도를 늘려 상체를 앞으로 숙임

        # PD 토크 연산
        joint_error = q_des - q_act
        torques = self.kp * joint_error - self.kd * v_act
        torques = np.clip(torques, -60.0, 60.0)

        # 텔레메트리 로깅
        self.log_time.append(sim_time)
        self.log_z.append(pos_z)
        self.log_pitch.append(pitch)
        self.log_gyro.append(np.linalg.norm(gyro[:2]))
        self.log_max_error.append(np.max(np.abs(joint_error)))

        return torques

def run_simulation_loop(model, data, controller, viewer=None, max_time=10.0):
    last_print_time = 0.0
    prev_sim_time = data.time

    # 1.0x 실시간 동기화를 위한 기준 시각
    wall_start = time.perf_counter()
    sim_start = data.time

    print(f"\n{Colors.BOLD}[TEST EXECUTION] Running Spawn Pose Hold Simulation...{Colors.RESET}")
    if viewer:
        print(f"  {Colors.CYAN}📺 3D MuJoCo 뷰어가 활성화되었습니다. (Space: 일시정지, Backspace: 리셋){Colors.RESET}")

    step = 0
    evaluation_done = False

    while True:
        if viewer and not viewer.is_running():
            print(f"  * 사용자에 의해 뷰어 창이 닫혔습니다.")
            break

        # [핵심] 뷰어 Reset 감지 (Backspace 또는 UI Reset 버튼 클릭 시)
        if data.time < prev_sim_time:
            print(f"\n  {Colors.YELLOW}↺ [RESET 감지] 뷰어 리셋이 감지되어 초기 스폰 키프레임으로 재동기화합니다.{Colors.RESET}")
            controller.reset()
            stand_key_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "stand")
            if stand_key_id != -1:
                mujoco.mj_resetDataKeyframe(model, data, stand_key_id)
            else:
                data.qpos[7:13] = controller.q_target.copy()
                data.qvel[:] = 0.0
            mujoco.mj_forward(model, data)
            evaluation_done = False
            wall_start = time.perf_counter()
            sim_start = data.time
            last_print_time = data.time
            prev_sim_time = data.time

        prev_sim_time = data.time

        # 1. 제어 토크 인가
        torques = controller.compute_torques(data)
        for i, act_id in enumerate(controller.act_ids):
            data.ctrl[act_id] = torques[i]

        # 2. 물리 1 스텝 전진
        mujoco.mj_step(model, data)
        step += 1

        # 3. 뷰어 화면 동기화 및 1.0x 실시간 속도 보정
        if viewer and (step % 5 == 0):
            viewer.sync()
            sim_elapsed = data.time - sim_start
            wall_elapsed = time.perf_counter() - wall_start
            sleep_time = sim_elapsed - wall_elapsed
            if sleep_time > 0:
                time.sleep(min(sleep_time, 0.05))

        # 4. 실시간 텍스트 상태 출력 (0.5초 주기)
        sim_time = data.time
        if (sim_time - last_print_time) >= 0.5:
            pos_x = data.xpos[controller.base_body_id][0]
            pos_z = data.xpos[controller.base_body_id][2]
            pitch_deg = math.degrees(controller.current_pitch)
            max_err = controller.log_max_error[-1] if len(controller.log_max_error) > 0 else 0.0
            print(f"  * [{sim_time:5.2f}s] 상태: {controller.state:<10} | Pitch={pitch_deg:+5.1f}° (수평불량: {controller.tilt_duration:3.1f}s/3s) | 위치: X={pos_x:5.3f}m, Z={pos_z:5.3f}m | 관절 오차={max_err:5.4f} rad")
            last_print_time = sim_time

        if controller.log_fall and not evaluation_done:
            pitch_deg = math.degrees(controller.current_pitch)
            print(f"  {Colors.RED}✗ [{sim_time:.2f}s] 로봇 수평 불량/전도 감지! (|Pitch| > 3.0° 지속시간 {controller.tilt_duration:.2f}s >= 3.0s, 현재 Pitch: {pitch_deg:.1f}°){Colors.RESET}")
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

    return controller, data

def evaluate_results(controller):
    print(f"\n{Colors.BOLD}[VERIFICATION RESULTS] Spawn Pose Hold Evaluation{Colors.RESET}")
    if controller.log_fall:
        print(f"  {Colors.RED}✗ [Check 1] 상체 수평 유지 및 전도 여부: 수평 불량 3초 초과 지속 / 전도 발생 [FAIL]{Colors.RESET}")
        return False
    else:
        print(f"  {Colors.GREEN}✓ [Check 1] 상체 수평 유지 및 전도 여부: 바닥과 수평(Pitch ≈ 0°) 유지 성공 [PASS]{Colors.RESET}")

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
    print(f"{Colors.BOLD}{Colors.CYAN}Phase 01-U01: Tron1 Spawn Pose Hold & Joint Control Test{Colors.RESET}")
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

    # 스폰 시 기립 자세로 지면 안착 (키프레임이 없을 경우 fallback)
    if stand_key_id == -1:
        data.qpos[7:13] = controller.q_target.copy()
    mujoco.mj_forward(model, data)

    print(f"  * 목표 관절 각도 (Target q): {np.round(controller.q_target, 3)}")

    # 뷰어 실행 모드 분기 (기본값: 3D 창 표시 + 텍스트 동시 출력)
    if not args.headless:
        try:
            with mujoco.viewer.launch_passive(model, data) as viewer:
                viewer.opt.geomgroup[0] = 1
                viewer.opt.geomgroup[1] = 1
                viewer.opt.geomgroup[2] = 1
                viewer.opt.geomgroup[3] = 1
                controller, final_data = run_simulation_loop(model, data, controller, viewer=viewer, max_time=args.max_time)
        except Exception as e:
            print(f"  {Colors.YELLOW}⚠ GUI 뷰어 실행 실패 ({e}) -> 텍스트 전용 모드로 전환{Colors.RESET}")
            controller, final_data = run_simulation_loop(model, data, controller, viewer=None, max_time=args.max_time)
    else:
        controller, final_data = run_simulation_loop(model, data, controller, viewer=None, max_time=args.max_time)

    return 0

if __name__ == "__main__":
    sys.exit(main())