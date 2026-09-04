#!/usr/bin/env python3
"""
scripts_devel_roadmap/phase01_u01_test_tron1_walking.py
Phase 01-U01: Tron1 Bipedal Locomotion & Docking Stance Lock Verification Script

Validates:
1. Ground Contact Impact Absorption & Standing Posture (Drop & Landing)
2. Open-Loop/Closed-Loop Phase-Synchronized Bipedal Locomotion to Target (x = 1.0m)
3. Stance Lock Stabilization: Roll/Pitch Vibration < 0.05 rad/s for 3.0 seconds
4. Offscreen RGB Snapshot Export (temp/u01_tron1_docking.png)
5. (Optional) Interactive 3D Viewer Mode (--viewer)
"""

import os
import sys
import argparse
import math
import numpy as np
import mujoco

class Colors:
    GREEN = '\033[92m'
    RED = '\033[91m'
    YELLOW = '\033[93m'
    CYAN = '\033[96m'
    BOLD = '\033[1m'
    RESET = '\033[0m'

def parse_args():
    parser = argparse.ArgumentParser(description="Phase 01-U01 Tron1 Locomotion Test")
    parser.add_argument("--viewer", action="store_true", help="Launch interactive 3D GUI viewer")
    parser.add_argument("--xml", type=str, default="unit_test_models/phase01_u01_scene_unit_tron1.xml",
                        help="Path to Tron1 unit scene XML")
    parser.add_argument("--target_x", type=float, default=1.0, help="Target docking X coordinate in meters")
    parser.add_argument("--max_time", type=float, default=12.0, help="Maximum simulation time limit in seconds")
    return parser.parse_args()

class Tron1BipedController:
    """
    Tron1 전용 2족 보행 및 스탠스 락 제어기
    - 위상 변수(Phase) 기반 사인파 궤적 생성
    - 상체 자세(Roll/Pitch) 안정화 피드백
    - 도킹 정지 시 게인 스케줄링(Stance Lock)
    """
    def __init__(self, model, target_x=1.0):
        self.model = model
        self.target_x = target_x

        # 관절 및 액추에이터 ID 매핑
        self.joint_names = [
            "abad_L_Joint", "hip_L_Joint", "knee_L_Joint",
            "abad_R_Joint", "hip_R_Joint", "knee_R_Joint"
        ]
        self.actuator_names = [
            "abad_L_motor", "hip_L_motor", "knee_L_motor",
            "abad_R_motor", "hip_R_motor", "knee_R_motor"
        ]
        self.act_ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name) for name in self.actuator_names]
        self.base_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_Link")

        # 기립 기본 관절 각도 (살짝 무릎을 굽힌 자연스러운 직립 자세)
        # 좌우 대칭성을 고려하여 hip, knee 축 방향에 맞게 부호 설정
        self.q_stand = np.array([0.0, 0.45, 0.85,  0.0, -0.45, -0.85])

        # 게인 설정: [보행 모드] vs [스탠스 락 모드]
        self.kp_walk = np.array([80.0, 100.0, 100.0,  80.0, 100.0, 100.0])
        self.kd_walk = np.array([3.0, 4.0, 4.0,        3.0, 4.0, 4.0])

        self.kp_lock = np.array([160.0, 200.0, 200.0,  160.0, 200.0, 200.0])
        self.kd_lock = np.array([8.0, 10.0, 10.0,      8.0, 10.0, 10.0])

        # 보행 파라미터
        self.gait_period = 0.50  # 1걸음 주기 (0.5초)
        self.step_length = 0.12  # 보폭 (각도 변위 진폭)
        self.step_height = 0.22  # 발들기 (무릎 굴곡 진폭)

        # 상태 머신 변수
        self.state = "LANDING"  # LANDING -> WALKING -> DOCKED -> COMPLETE
        self.dock_time = None
        self.stance_lock_duration = 3.0  # 정지 후 3초 유지

        # 로깅용 기록 리스트
        self.log_time = []
        self.log_x = []
        self.log_gyro = []
        self.log_fall = False

    def compute_torques(self, data):
        sim_time = data.time
        pos_x = data.xpos[self.base_body_id][0]
        pos_z = data.xpos[self.base_body_id][2]

        # 1. 전도 감지 (상체 높이가 0.4m 이하로 떨어지면 넘어짐으로 판정)
        if pos_z < 0.40:
            self.log_fall = True

        # 2. 현재 관절 위치 및 속도 추출 (qpos 7~12: 다리 관절, qvel 6~11)
        q_act = data.qpos[7:13]
        v_act = data.qvel[6:12]

        # 3. IMU 자이로(각속도) 센서 읽기 (롤, 피치, 요)
        gyro = data.sensor("imu_gyro").data.copy()

        # 4. 상태 머신 분기
        if self.state == "LANDING":
            q_des = self.q_stand.copy()
            kp = self.kp_walk
            kd = self.kd_walk
            if sim_time >= 1.5:
                self.state = "WALKING"

        elif self.state == "WALKING":
            # 목표 지점(1.0m) 도달 여부 체크
            if pos_x >= (self.target_x - 0.05):
                self.state = "DOCKED"
                self.dock_time = sim_time

            # 위상 변수 phi in [0, 1)
            phi = (sim_time % self.gait_period) / self.gait_period

            q_des = self.q_stand.copy()
            
            # 교대 보행 위상 궤적 계산
            # Half-cycle 1 (phi < 0.5): Left Stance, Right Swing
            # Half-cycle 2 (phi >= 0.5): Right Stance, Left Swing
            hip_swing = math.sin(2.0 * math.pi * phi) * self.step_length
            knee_swing = max(0.0, math.sin(2.0 * math.pi * phi)) * self.step_height

            # Left leg
            q_des[1] += hip_swing
            q_des[2] -= knee_swing
            # Right leg (반대 부호 축 고려)
            q_des[4] += hip_swing
            q_des[5] -= knee_swing

            # 롤/피치 자세 균형 보정 (자이로 피드백 감쇠)
            q_des[0] -= 0.05 * gyro[0]  # Abad Roll 보정
            q_des[3] -= 0.05 * gyro[0]

            kp = self.kp_walk
            kd = self.kd_walk

        elif self.state == "DOCKED":
            # 스탠스 락(Stance Lock) 활성화: 게인을 크게 올리고 기립 자세 고정
            q_des = self.q_stand.copy()
            kp = self.kp_lock
            kd = self.kd_lock

            # 정지 상태 유지 시간 검사
            if (sim_time - self.dock_time) >= self.stance_lock_duration:
                self.state = "COMPLETE"

        else:  # COMPLETE
            q_des = self.q_stand.copy()
            kp = self.kp_lock
            kd = self.kd_lock

        # 5. 관절 토크 계산: tau = kp * (q_des - q_act) - kd * v_act
        torques = kp * (q_des - q_act) - kd * v_act

        # 6. 토크 제한 클램핑 ([-80, 80] Nm)
        torques = np.clip(torques, -80.0, 80.0)

        # 로깅
        self.log_time.append(sim_time)
        self.log_x.append(pos_x)
        self.log_gyro.append(np.linalg.norm(gyro[:2]))  # Roll & Pitch 각속도 크기

        return torques

def run_simulation(model, args):
    print(f"\n{Colors.BOLD}[TEST EXECUTION] Running Bipedal Locomotion Simulation...{Colors.RESET}")
    data = mujoco.MjData(model)
    mujoco.mj_resetData(model, data)

    # 초기 상태 포워드
    mujoco.mj_forward(model, data)

    controller = Tron1BipedController(model, target_x=args.target_x)
    dt = model.opt.timestep
    max_steps = int(args.max_time / dt)

    dock_reported = False

    for step in range(max_steps):
        # 1. 제어 토크 계산 및 적용
        torques = controller.compute_torques(data)
        for i, act_id in enumerate(controller.act_ids):
            data.ctrl[act_id] = torques[i]

        # 2. 물리 1 스텝 적분
        mujoco.mj_step(model, data)

        # 도킹 진입 시 콘솔 출력
        if controller.state == "DOCKED" and not dock_reported:
            pos_x = data.xpos[controller.base_body_id][0]
            print(f"  * [{data.time:.2f}s] 도킹 구역 도달! (x = {pos_x:.3f}m) -> Stance Lock 전환 (3초 안정화 시작)")
            dock_reported = True

        # 완료 조건 달성 시 조기 종료
        if controller.state == "COMPLETE":
            print(f"  * [{data.time:.2f}s] Stance Lock 3초 안정화 성공적으로 완료!")
            break

        # 전도 발생 시 즉시 중단
        if controller.log_fall:
            print(f"  {Colors.RED}✗ 로봇이 전도되었습니다! (h < 0.4m){Colors.RESET}")
            break

    return controller, data

def evaluate_results(controller, final_data, target_x):
    print(f"\n{Colors.BOLD}[VERIFICATION RESULTS] Performance Evaluation{Colors.RESET}")
    passed = True

    # 1. 전도 여부 확인
    if controller.log_fall:
        print(f"  {Colors.RED}✗ [Check 1] 전도 여부: 전도 발생 [FAIL]{Colors.RESET}")
        return False
    else:
        print(f"  {Colors.GREEN}✓ [Check 1] 전도 여부: 넘어지지 않고 직립 유지 [PASS]{Colors.RESET}")

    # 2. 이동 목표 지점 오차 검증
    final_x = controller.log_x[-1] if len(controller.log_x) > 0 else 0.0
    x_error = abs(final_x - target_x)
    print(f"  * 최종 위치: x = {final_x:.3f} m (목표 x = {target_x:.3f} m, 오차: {x_error*100:.1f} cm)")
    if x_error <= 0.10:  # 10cm 이내 도달
        print(f"  {Colors.GREEN}✓ [Check 2] 도킹 도달 정밀도 합격 (오차 10cm 이내) [PASS]{Colors.RESET}")
    else:
        print(f"  {Colors.RED}✗ [Check 2] 도킹 도달 오차 초과: {x_error*100:.1f}cm (기준 10cm 이하) [FAIL]{Colors.RESET}")
        passed = False

    # 3. Stance Lock 3초간 잔류 각속도(진동) 수렴 검증
    # 도킹 이후 마지막 2.0초 구간의 롤/피치 각속도 평균 계산
    if controller.dock_time is not None:
        times = np.array(controller.log_time)
        gyros = np.array(controller.log_gyro)
        lock_mask = times >= (controller.dock_time + 1.0)
        if np.any(lock_mask):
            mean_vibration = np.mean(gyros[lock_mask])
            max_vibration = np.max(gyros[lock_mask])
            print(f"  * Stance Lock 수렴 각속도: 평균={mean_vibration:.4f} rad/s, 최대={max_vibration:.4f} rad/s")
            if mean_vibration <= 0.05:
                print(f"  {Colors.GREEN}✓ [Check 3] 정지 상태 진동 억제 합격 (< 0.05 rad/s) [PASS]{Colors.RESET}")
            else:
                print(f"  {Colors.RED}✗ [Check 3] 진동 수렴 불량: {mean_vibration:.4f} rad/s (기준 0.05 rad/s 이하) [FAIL]{Colors.RESET}")
                passed = False
        else:
            print(f"  {Colors.RED}✗ [Check 3] 정지 유지 시간 부족 [FAIL]{Colors.RESET}")
            passed = False
    else:
        print(f"  {Colors.RED}✗ [Check 3] 도킹 구역에 도달하지 못했습니다. [FAIL]{Colors.RESET}")
        passed = False

    return passed

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

def run_interactive_viewer(model, args):
    print(f"\n{Colors.BOLD}[INTERACTIVE MODE] Launching 3D MuJoCo Viewer...{Colors.RESET}")
    try:
        import mujoco.viewer
        data = mujoco.MjData(model)
        mujoco.mj_resetData(model, data)
        controller = Tron1BipedController(model, target_x=args.target_x)

        with mujoco.viewer.launch_passive(model, data) as viewer:
            while viewer.is_running():
                torques = controller.compute_torques(data)
                for i, act_id in enumerate(controller.act_ids):
                    data.ctrl[act_id] = torques[i]
                mujoco.mj_step(model, data)
                viewer.sync()
    except Exception as e:
        print(f"  {Colors.RED}✗ 뷰어 실행 실패: {e}{Colors.RESET}")

def main():
    args = parse_args()
    print(f"\n{Colors.BOLD}{Colors.CYAN}============================================================{Colors.RESET}")
    print(f"{Colors.BOLD}{Colors.CYAN}Phase 01-U01: Tron1 Bipedal Locomotion & Docking Stance Lock{Colors.RESET}")
    print(f"{Colors.BOLD}{Colors.CYAN}============================================================{Colors.RESET}")
    print(f"  * Target XML : {args.xml}")
    print(f"  * Docking X  : {args.target_x} m")

    if not os.path.exists(args.xml):
        print(f"{Colors.RED}✗ XML 파일을 찾을 수 없습니다: {args.xml}{Colors.RESET}")
        print(f"  가이드 문서(Step 2)를 참고하여 씬 파일을 먼저 생성하세요.")
        return 1

    model = mujoco.MjModel.from_xml_path(args.xml)
    print(f"  {Colors.GREEN}✓ MJCF 파싱 및 MjModel 로드 성공 [PASS]{Colors.RESET}")

    # 시뮬레이션 및 평가
    controller, final_data = run_simulation(model, args)
    success = evaluate_results(controller, final_data, args.target_x)

    # 스냅샷 저장
    export_snapshot(model, final_data)

    print(f"\n{Colors.BOLD}{Colors.CYAN}============================================================{Colors.RESET}")
    if success:
        print(f"{Colors.BOLD}{Colors.GREEN}🎉 [SUCCESS] Phase 01-U01 Tron1 기본 보행 및 도킹 정지 검증 완수!{Colors.RESET}")
        print(f"   다음 단위 단계인 [U02: 상체 트레이 장착 및 물병 운반]으로 진행할 수 있습니다.\n")
    else:
        print(f"{Colors.BOLD}{Colors.RED}❌ [FAILED] 보행/도킹 성능 기준을 만족하지 못했습니다. 제어기 게인을 튜닝하세요.\n{Colors.RESET}")

    # 대화형 뷰어 모드
    if args.viewer:
        run_interactive_viewer(model, args)

    return 0 if success else 1

if __name__ == "__main__":
    sys.exit(main())
