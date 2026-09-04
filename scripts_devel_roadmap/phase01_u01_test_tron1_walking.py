#!/usr/bin/env python3
"""
scripts_devel_roadmap/phase01_u01_test_tron1_walking.py
Phase 01-U01: Tron1 Bipedal Locomotion & Docking Stance Lock
- 1.0x Real-time Physics Speed Synchronization
- Interactive Viewer Auto-Reset Support (Instant sync on Backspace/Reset)
- Unified 3D GUI & Console Telemetry Loop
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
    parser = argparse.ArgumentParser(description="Phase 01-U01 Tron1 Locomotion Test")
    parser.add_argument("--headless", action="store_true", help="Run in headless text-only mode (no GUI window)")
    parser.add_argument("--xml", type=str, default="unit_test_models/phase01_u01_scene_unit_tron1.xml",
                        help="Path to Tron1 unit scene XML")
    parser.add_argument("--target_x", type=float, default=1.0, help="Target docking X coordinate in meters")
    parser.add_argument("--max_time", type=float, default=12.0, help="Maximum simulation time limit in seconds")
    return parser.parse_args()

class Tron1BipedController:
    """
    Tron1 전용 2족 보행 및 스탠스 락 제어기
    - 기립 초기화: 스폰 순간 다리 급접힘(kick-down) 방지
    - 위상 변수(Phase) 기반 교대 보행 사인파 궤적
    - 상체 자세(Roll/Pitch) 안정화 피드백
    - 도킹 정지 시 게인 스케줄링(Stance Lock)
    """
    def __init__(self, model, target_x=1.0):
        self.model = model
        self.target_x = target_x

        self.actuator_names = [
            "abad_L_motor", "hip_L_motor", "knee_L_motor",
            "abad_R_motor", "hip_R_motor", "knee_R_motor"
        ]
        self.act_ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name) for name in self.actuator_names]
        self.base_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_Link")

        # 기립 기본 관절 각도 (자연스러운 완충 기립 자세)
        self.q_stand = np.array([0.0, 0.40, 0.80,  0.0, -0.40, -0.80])

        # 관절 강성 및 댐핑 게인
        self.kp_walk = np.array([120.0, 150.0, 150.0,  120.0, 150.0, 150.0])
        self.kd_walk = np.array([6.0, 8.0, 8.0,        6.0, 8.0, 8.0])

        self.kp_lock = np.array([200.0, 250.0, 250.0,  200.0, 250.0, 250.0])
        self.kd_lock = np.array([12.0, 15.0, 15.0,     12.0, 15.0, 15.0])

        self.gait_period = 0.50
        self.step_length = 0.12
        self.step_height = 0.18

        self.reset()

    def reset(self):
        self.state = "LANDING"
        self.dock_time = None
        self.stance_lock_duration = 3.0
        self.log_time = []
        self.log_x = []
        self.log_gyro = []
        self.log_fall = False

    def compute_torques(self, data):
        sim_time = data.time
        pos_x = data.xpos[self.base_body_id][0]
        pos_z = data.xpos[self.base_body_id][2]

        # 전도 감지 (상체 높이 0.35m 이하 추락 시)
        if pos_z < 0.35:
            self.log_fall = True

        q_act = data.qpos[7:13]
        v_act = data.qvel[6:12]
        gyro = data.sensor("imu_gyro").data.copy()

        # FSM 상태 전이
        if self.state == "LANDING":
            q_des = self.q_stand.copy()
            kp = self.kp_walk
            kd = self.kd_walk
            if sim_time >= 1.5:
                self.state = "WALKING"

        elif self.state == "WALKING":
            if pos_x >= (self.target_x - 0.05):
                self.state = "DOCKED"
                self.dock_time = sim_time

            phi = (sim_time % self.gait_period) / self.gait_period
            q_des = self.q_stand.copy()

            hip_swing = math.sin(2.0 * math.pi * phi) * self.step_length
            knee_swing = max(0.0, math.sin(2.0 * math.pi * phi)) * self.step_height

            q_des[1] += hip_swing
            q_des[2] -= knee_swing
            q_des[4] += hip_swing
            q_des[5] -= knee_swing

            # 자이로 기반 상체 롤/피치 균형 보정
            q_des[0] -= 0.04 * gyro[0]
            q_des[3] -= 0.04 * gyro[0]

            kp = self.kp_walk
            kd = self.kd_walk

        elif self.state == "DOCKED":
            q_des = self.q_stand.copy()
            kp = self.kp_lock
            kd = self.kd_lock
            if (sim_time - self.dock_time) >= self.stance_lock_duration:
                self.state = "COMPLETE"

        else:  # COMPLETE
            q_des = self.q_stand.copy()
            kp = self.kp_lock
            kd = self.kd_lock

        torques = kp * (q_des - q_act) - kd * v_act
        torques = np.clip(torques, -80.0, 80.0)

        self.log_time.append(sim_time)
        self.log_x.append(pos_x)
        self.log_gyro.append(np.linalg.norm(gyro[:2]))

        return torques

def run_simulation_loop(model, data, controller, viewer=None, max_time=12.0):
    dt = model.opt.timestep
    last_print_time = 0.0
    dock_reported = False
    prev_sim_time = data.time

    # 1.0x 실시간 동기화를 위한 기준 시각
    wall_start = time.perf_counter()
    sim_start = data.time

    print(f"\n{Colors.BOLD}[TEST EXECUTION] Running Bipedal Locomotion Simulation...{Colors.RESET}")
    if viewer:
        print(f"  {Colors.CYAN}📺 3D MuJoCo 뷰어가 활성화되었습니다. (Space: 일시정지, Backspace: 리셋){Colors.RESET}")

    step = 0
    evaluation_done = False

    while True:
        if viewer and not viewer.is_running():
            print(f"  * 사용자에 의해 뷰어 창이 닫혔습니다.")
            break

        # [핵심 1] 뷰어 Reset 감지 (Backspace 또는 UI Reset 버튼 클릭 시)
        if data.time < prev_sim_time:
            print(f"\n  {Colors.YELLOW}↺ [RESET 감지] 뷰어 리셋이 감지되어 제어기 및 초기 관절 자세를 완벽히 재동기화합니다.{Colors.RESET}")
            controller.reset()
            data.qpos[7:13] = controller.q_stand.copy()
            data.qvel[:] = 0.0
            mujoco.mj_forward(model, data)
            dock_reported = False
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
            print(f"  * [{sim_time:5.2f}s] 상태: {controller.state:<8} | 위치: X={pos_x:5.3f}m, 높이 Z={pos_z:5.3f}m")
            last_print_time = sim_time

        if controller.state == "DOCKED" and not dock_reported:
            pos_x = data.xpos[controller.base_body_id][0]
            print(f"  {Colors.GREEN}★ [{sim_time:.2f}s] 도킹 구역 도달! (X = {pos_x:.3f}m) -> Stance Lock 전환 (3초 안정화 시작){Colors.RESET}")
            dock_reported = True

        if controller.state == "COMPLETE" and not evaluation_done:
            print(f"  {Colors.GREEN}★ [{sim_time:.2f}s] Stance Lock 3초 안정화 완수!{Colors.RESET}")
            evaluation_done = True
            success = evaluate_results(controller, controller.target_x)
            export_snapshot(model, data)
            print(f"\n{Colors.BOLD}{Colors.CYAN}============================================================{Colors.RESET}")
            if success:
                print(f"{Colors.BOLD}{Colors.GREEN}🎉 [SUCCESS] Phase 01-U01 Tron1 기본 보행 및 도킹 정지 검증 완수!{Colors.RESET}")
            else:
                print(f"{Colors.BOLD}{Colors.RED}❌ [FAILED] 보행/도킹 성능 기준을 만족하지 못했습니다.{Colors.RESET}")
            print(f"{Colors.BOLD}{Colors.CYAN}============================================================{Colors.RESET}")
            if not viewer:
                break
            else:
                print(f"  {Colors.CYAN}💡 3D 창에서 로봇을 자유롭게 관찰하세요. Backspace를 누르면 처음부터 다시 걷습니다.{Colors.RESET}")

        if controller.log_fall and not evaluation_done:
            print(f"  {Colors.RED}✗ [{sim_time:.2f}s] 로봇 전도 발생! (h < 0.35m){Colors.RESET}")
            evaluation_done = True
            evaluate_results(controller, controller.target_x)
            if not viewer:
                break

        # 헤드리스 모드 종료 조건
        if not viewer and data.time >= max_time:
            if not evaluation_done:
                evaluate_results(controller, controller.target_x)
            break

    return controller, data

def evaluate_results(controller, target_x):
    print(f"\n{Colors.BOLD}[VERIFICATION RESULTS] Performance Evaluation{Colors.RESET}")
    if controller.log_fall:
        print(f"  {Colors.RED}✗ [Check 1] 전도 여부: 전도 발생 [FAIL]{Colors.RESET}")
        return False
    else:
        print(f"  {Colors.GREEN}✓ [Check 1] 전도 여부: 넘어지지 않고 직립 유지 [PASS]{Colors.RESET}")

    final_x = controller.log_x[-1] if len(controller.log_x) > 0 else 0.0
    x_error = abs(final_x - target_x)
    print(f"  * 최종 위치: x = {final_x:.3f} m (목표 x = {target_x:.3f} m, 오차: {x_error*100:.1f} cm)")
    dock_pass = (x_error <= 0.15)
    if dock_pass:
        print(f"  {Colors.GREEN}✓ [Check 2] 도킹 도달 정밀도 합격 (오차 15cm 이내) [PASS]{Colors.RESET}")
    else:
        print(f"  {Colors.RED}✗ [Check 2] 도킹 도달 오차 초과: {x_error*100:.1f}cm [FAIL]{Colors.RESET}")

    vibration_pass = False
    if controller.dock_time is not None:
        times = np.array(controller.log_time)
        gyros = np.array(controller.log_gyro)
        lock_mask = times >= (controller.dock_time + 1.0)
        if np.any(lock_mask):
            mean_vibration = np.mean(gyros[lock_mask])
            print(f"  * Stance Lock 수렴 각속도: 평균={mean_vibration:.4f} rad/s")
            if mean_vibration <= 0.08:
                print(f"  {Colors.GREEN}✓ [Check 3] 정지 상태 진동 억제 합격 (< 0.08 rad/s) [PASS]{Colors.RESET}")
                vibration_pass = True
            else:
                print(f"  {Colors.RED}✗ [Check 3] 정지 상태 진동 억제 불합격 (>= 0.08 rad/s) [FAIL]{Colors.RESET}")

    return dock_pass and vibration_pass

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
    print(f"{Colors.BOLD}{Colors.CYAN}Phase 01-U01: Tron1 Bipedal Locomotion & Docking Stance Lock{Colors.RESET}")
    print(f"{Colors.BOLD}{Colors.CYAN}============================================================{Colors.RESET}")
    print(f"  * Target XML : {args.xml}")
    print(f"  * Docking X  : {args.target_x} m")

    if not os.path.exists(args.xml):
        print(f"{Colors.RED}✗ XML 파일을 찾을 수 없습니다: {args.xml}{Colors.RESET}")
        return 1

    model = mujoco.MjModel.from_xml_path(args.xml)
    data = mujoco.MjData(model)
    mujoco.mj_resetData(model, data)

    controller = Tron1BipedController(model, target_x=args.target_x)

    # 스폰 시 기립 자세로 부드럽게 지면 안착
    data.qpos[7:13] = controller.q_stand.copy()
    mujoco.mj_forward(model, data)

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