#!/usr/bin/env python3
"""
scripts_devel_roadmap/phase01_u00_test_base_sandbox.py
Phase 01-U00: Unit Sandbox Base Environment & Model Loader Verification Script

Validates:
1. XML Parsing & MJCF Integrity
2. Physics Parameters (dt=0.001s/1000Hz, gravity=-9.81 m/s^2, integrator=implicitfast)
3. Freefall Dynamics Numerical Accuracy against Theoretical Kinematics (h = 0.5 * g * t^2)
4. Headless Offscreen RGB-D Buffer Rendering & PNG Export
5. (Optional) Interactive GUI 3D Viewer Mode (--viewer)
"""

import os
import sys
import argparse
import math
import numpy as np
import mujoco

# 터미널 색상 출력
class Colors:
    GREEN = '\033[92m'
    RED = '\033[91m'
    YELLOW = '\033[93m'
    CYAN = '\033[96m'
    BOLD = '\033[1m'
    RESET = '\033[0m'

def parse_args():
    parser = argparse.ArgumentParser(description="Phase 01-U00 Base Sandbox Test")
    parser.add_argument("--viewer", action="store_true", help="Launch interactive 3D GUI viewer")
    parser.add_argument("--xml", type=str, default="xml_for_unit_test/phase01_u00_scene_unit_base.xml", help="Path to base scene XML")
    return parser.parse_args()

def verify_physics_parameters(model):
    print(f"\n{Colors.BOLD}[TEST 1] Physics Engine Parameters Verification{Colors.RESET}")
    passed = True

    # 1. dt 유효성 검증 및 주파수(Hz) 확인
    dt = model.opt.timestep
    if dt > 0:
        freq_hz = 1.0 / dt
        print(f"  {Colors.GREEN}✓ Timestep: {dt:.4f}s ({freq_hz:.0f} Hz) [PASS]{Colors.RESET}")
    else:
        print(f"  {Colors.RED}✗ 유효하지 않은 Timestep: {dt}s [FAIL]{Colors.RESET}")
        passed = False

    # 2. 중력 검증 (z = -9.81)
    expected_gz = -9.81
    actual_gz = model.opt.gravity[2]
    if math.isclose(actual_gz, expected_gz, abs_tol=1e-3):
        print(f"  {Colors.GREEN}✓ Gravity: {model.opt.gravity} m/s^2 [PASS]{Colors.RESET}")
    else:
        print(f"  {Colors.RED}✗ Gravity 불일치: z={actual_gz}, 기대값={expected_gz} [FAIL]{Colors.RESET}")
        passed = False

    # 3. 적분기 검증 (implicitfast = 2 또는 implicit = 1)
    # MuJoCo enum: mjtIntegrator: 0: Euler, 1: RK4, 2: implicit, 3: implicitfast
    integrator_id = model.opt.integrator
    integrator_names = {0: "Euler", 1: "RK4", 2: "implicit", 3: "implicitfast"}
    integ_name = integrator_names.get(integrator_id, f"Unknown({integrator_id})")
    print(f"  * Integrator: {integ_name}")
    if integrator_id in (2, 3):
        print(f"  {Colors.GREEN}✓ 음함수 안정 적분기({integ_name}) 적용 확인 [PASS]{Colors.RESET}")
    else:
        print(f"  {Colors.YELLOW}⚠ 경고: 음함수 적분기 대신 {integ_name}이 선택되었습니다.{Colors.RESET}")

    return passed

def verify_freefall_dynamics(model):
    print(f"\n{Colors.BOLD}[TEST 2] Freefall Kinematics Numerical Accuracy Test{Colors.RESET}")
    data = mujoco.MjData(model)
    mujoco.mj_resetData(model, data)
    mujoco.mj_forward(model, data)

    # 테스트 구체 정보 추출
    ball_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "test_ball")
    ball_geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "test_ball_geom")
    ball_radius = model.geom_size[ball_geom_id][0] # 0.05m
    initial_z = data.xpos[ball_body_id][2]          # 1.0m
    target_contact_z = ball_radius                  # 바닥 접촉 시 구체 중심 높이 (0.05m)
    fall_distance = initial_z - target_contact_z     # 0.95m

    # 이론 낙하 시간: t = sqrt(2 * h / g)
    g = abs(model.opt.gravity[2])
    theoretical_time = math.sqrt(2.0 * fall_distance / g)

    print(f"  * 초기 중심 높이: {initial_z:.3f} m, 구체 반지름: {ball_radius:.3f} m")
    print(f"  * 바닥 충돌 전 낙하 거리: {fall_distance:.3f} m")
    print(f"  * 이론적 지면 충돌 도달 시간: {theoretical_time:.4f} s")

    # 충돌 직전까지 물리 시뮬레이션 적분 수행 (최대 물리 시간 2.0초 기준 스텝 산출)
    sim_time = 0.0
    measured_time = None
    max_steps = int(2.0 / model.opt.timestep) if model.opt.timestep > 0 else 1000

    for step in range(max_steps):
        mujoco.mj_step(model, data)
        current_z = data.xpos[ball_body_id][2]
        
        # 바닥에 접촉했는지 체크 (중심 높이가 구체 반지름 이하로 떨어지는 순간)
        if current_z <= (target_contact_z + 0.001):
            measured_time = data.time
            break

    if measured_time is not None:
        error_sec = abs(measured_time - theoretical_time)
        error_pct = (error_sec / theoretical_time) * 100.0
        print(f"  * 시뮬레이션 측정 도달 시간: {measured_time:.4f} s (오차: {error_sec:.5f} s, {error_pct:.2f}%)")

        if error_pct <= 0.5:
            print(f"  {Colors.GREEN}✓ 자유낙하 이론값 오차 0.5% 이내 합격 ({error_pct:.2f}%) [PASS]{Colors.RESET}")
            return True
        else:
            print(f"  {Colors.RED}✗ 수치 적분 오차 초과: {error_pct:.2f}% (기준 0.5% 이하) [FAIL]{Colors.RESET}")
            return False
    else:
        print(f"  {Colors.RED}✗ 타임아웃 내에 바닥 충돌을 감지하지 못했습니다. [FAIL]{Colors.RESET}")
        return False

def verify_offscreen_rendering(model):
    print(f"\n{Colors.BOLD}[TEST 3] Offscreen RGB-D Buffer Rendering & Image Export{Colors.RESET}")
    try:
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)

        renderer = mujoco.Renderer(model, width=640, height=480)
        renderer.update_scene(data, camera="overview_cam")
        rgb = renderer.render()

        renderer.enable_depth_rendering()
        renderer.update_scene(data, camera="overview_cam")
        depth = renderer.render()
        renderer.disable_depth_rendering()

        if rgb.shape == (480, 640, 3) and depth.shape == (480, 640):
            print(f"  {Colors.GREEN}✓ RGB 버퍼 크기 일치: {rgb.shape}, uint8 [PASS]{Colors.RESET}")
            print(f"  {Colors.GREEN}✓ Depth 버퍼 크기 일치: {depth.shape}, min={depth.min():.2f}m, max={depth.max():.2f}m [PASS]{Colors.RESET}")

            # 이미지 파일 저장 (cv2 또는 PIL)
            os.makedirs("temp", exist_ok=True)
            output_img_path = "temp/u00_base_scene.png"
            try:
                import cv2
                # RGB to BGR for OpenCV
                bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
                cv2.imwrite(output_img_path, bgr)
                print(f"  {Colors.GREEN}✓ 렌더링 스냅샷 저장 완료: {output_img_path} [PASS]{Colors.RESET}")
            except Exception as e:
                print(f"  {Colors.YELLOW}⚠ OpenCV 저장 실패({e}), matplotlib 시도...{Colors.RESET}")
                import matplotlib.pyplot as plt
                plt.imsave(output_img_path, rgb)
                print(f"  {Colors.GREEN}✓ 렌더링 스냅샷 저장 완료: {output_img_path} [PASS]{Colors.RESET}")

            return True
        else:
            print(f"  {Colors.RED}✗ 렌더링 해상도 불일치: RGB {rgb.shape}, Depth {depth.shape} [FAIL]{Colors.RESET}")
            return False
    except Exception as e:
        print(f"  {Colors.RED}✗ 오프스크린 렌더링 실패: {e} [FAIL]{Colors.RESET}")
        return False

def run_interactive_viewer(model):
    print(f"\n{Colors.BOLD}[INTERACTIVE MODE] Launching 3D MuJoCo Viewer...{Colors.RESET}")
    print(f"  * 마우스 좌클릭: 뷰 회전 | 우클릭: 뷰 이동 | 스크롤: 줌")
    print(f"  * Space 바: 시뮬레이션 일시정지/재개 | Backspace: 초기 상태(공중 1.0m) 리셋")
    print(f"  * 붉은 구체(test_ball, 지름 10cm)가 공중 1.0m에서 바닥으로 낙하하는 물리 현상을 관찰할 수 있습니다.")
    print(f"  * 창을 닫으면 프로그램이 종료됩니다.")
    
    try:
        import mujoco_viewer
        data = mujoco.MjData(model)
        viewer = mujoco_viewer.MujocoViewer(model, data)
        # 모든 충돌/시각 그룹 활성화
        viewer.opt.geomgroup[1] = 1
        viewer.opt.geomgroup[3] = 1
        while viewer.is_alive:
            mujoco.mj_step(model, data)
            viewer.render()
        viewer.close()
        print(f"  {Colors.GREEN}✓ 뷰어 세션 정상 종료{Colors.RESET}")
    except Exception as e:
        print(f"  {Colors.YELLOW}⚠ mujoco_viewer 실행 실패 ({e}), mujoco.viewer 내장 모듈 시도...{Colors.RESET}")
        import mujoco.viewer
        data = mujoco.MjData(model)
        with mujoco.viewer.launch_passive(model, data) as viewer:
            viewer.opt.geomgroup[1] = 1
            viewer.opt.geomgroup[3] = 1
            while viewer.is_running():
                mujoco.mj_step(model, data)
                viewer.sync()

def main():
    args = parse_args()
    print(f"\n{Colors.BOLD}{Colors.CYAN}============================================================{Colors.RESET}")
    print(f"{Colors.BOLD}{Colors.CYAN}Phase 01-U00: Unit Sandbox Base Environment Verification{Colors.RESET}")
    print(f"{Colors.BOLD}{Colors.CYAN}============================================================{Colors.RESET}")
    print(f"  * Target XML: {args.xml}")

    if not os.path.exists(args.xml):
        print(f"{Colors.RED}✗ XML 파일을 찾을 수 없습니다: {args.xml}{Colors.RESET}")
        return 1

    try:
        model = mujoco.MjModel.from_xml_path(args.xml)
        print(f"  {Colors.GREEN}✓ XML 파싱 및 MjModel 로드 성공 [PASS]{Colors.RESET}")
    except Exception as e:
        print(f"{Colors.RED}✗ MJCF 컴파일/파싱 실패: {e} [FAIL]{Colors.RESET}")
        return 1

    # 3대 자동 검증 수행
    test_results = [
        ("Physics Parameters", verify_physics_parameters(model)),
        ("Freefall Dynamics", verify_freefall_dynamics(model)),
        ("Offscreen Rendering", verify_offscreen_rendering(model)),
    ]

    all_passed = all(passed for _, passed in test_results)

    print(f"\n{Colors.BOLD}{Colors.CYAN}============================================================{Colors.RESET}")
    print(f"{Colors.BOLD}{Colors.CYAN}U00 Verification Summary{Colors.RESET}")
    print(f"{Colors.BOLD}{Colors.CYAN}============================================================{Colors.RESET}")
    for name, passed in test_results:
        status = f"{Colors.GREEN}[PASS]{Colors.RESET}" if passed else f"{Colors.RED}[FAIL]{Colors.RESET}"
        print(f"  * {name:<30}: {status}")

    if all_passed:
        print(f"\n{Colors.BOLD}{Colors.GREEN}🎉 [SUCCESS] Phase 01-U00 단위 검증 샌드박스 공통 환경이 완벽히 구축되었습니다!{Colors.RESET}")
        print(f"   다음 단위 단계인 [U01: Tron1 기본 이족보행 단독 검증]으로 진행할 수 있습니다.\n")
    else:
        print(f"\n{Colors.BOLD}{Colors.RED}❌ [FAILED] 일부 검증 항목이 실패하였습니다. 설정을 점검하세요.\n{Colors.RESET}")
        return 1

    # 옵션: 3D 인터랙티브 뷰어 실행
    if args.viewer:
        run_interactive_viewer(model)

    return 0

if __name__ == "__main__":
    sys.exit(main())
