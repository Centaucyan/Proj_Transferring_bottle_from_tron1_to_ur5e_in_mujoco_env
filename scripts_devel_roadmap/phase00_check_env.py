#!/usr/bin/env python3
"""
scripts_devel_roadmap/phase00_check_env.py
Phase 00 Automated Environment Verification Script
Validates:
1. Python Runtime (3.10.x in Conda environment)
2. ROS 2 Environment (Humble setup sourced, environment variables)
3. CXXABI & libstdc++ Symbol Compatibility (GLIBCXX_3.4.30+)
4. Sequential Imports of Heterogeneous Core Packages:
   rclpy, mujoco, open3d, cv2, tf2_ros, py_trees, typeguard, pydot
5. MuJoCo Headless & Offscreen RGB-D Rendering
"""

import sys
import os
import subprocess

class Colors:
    GREEN = '\033[92m'
    RED = '\033[91m'
    YELLOW = '\033[93m'
    CYAN = '\033[96m'
    BOLD = '\033[1m'
    RESET = '\033[0m'

def print_header(title):
    print(f"\n{Colors.BOLD}{Colors.CYAN}============================================================{Colors.RESET}")
    print(f"{Colors.BOLD}{Colors.CYAN}{title}{Colors.RESET}")
    print(f"{Colors.BOLD}{Colors.CYAN}============================================================{Colors.RESET}")

def check_python_runtime():
    print(f"\n{Colors.BOLD}[CHECK 1] Python Runtime Verification{Colors.RESET}")
    py_ver = sys.version_info
    print(f"  * Python Executable: {sys.executable}")
    print(f"  * Python Version   : {py_ver.major}.{py_ver.minor}.{py_ver.micro}")

    passed = True
    if py_ver.major == 3 and py_ver.minor == 10:
        print(f"  {Colors.GREEN}✓ Python 3.10.x 일치 확인 (ROS 2 Humble 호환) [PASS]{Colors.RESET}")
    else:
        print(f"  {Colors.RED}✗ Python 버전 불일치: 3.10.x여야 하지만 {py_ver.major}.{py_ver.minor}입니다. [FAIL]{Colors.RESET}")
        passed = False

    conda_prefix = os.environ.get("CONDA_PREFIX", "")
    if "transfer_bottle_by_tron1_py3_10" in conda_prefix:
        print(f"  {Colors.GREEN}✓ Conda 환경 활성화 확인: {conda_prefix} [PASS]{Colors.RESET}")
    else:
        print(f"  {Colors.YELLOW}⚠ 현재 활성화된 Conda 환경({conda_prefix})이 'transfer_bottle_by_tron1_py3_10'이 아닙니다.{Colors.RESET}")

    return passed

def check_ros2_env():
    print(f"\n{Colors.BOLD}[CHECK 2] ROS 2 Humble Environment Verification{Colors.RESET}")
    distro = os.environ.get("ROS_DISTRO", "")
    ament_prefix = os.environ.get("AMENT_PREFIX_PATH", "")
    pythonpath = os.environ.get("PYTHONPATH", "")

    print(f"  * ROS_DISTRO       : {distro if distro else '(설정되지 않음)'}")
    print(f"  * AMENT_PREFIX_PATH: {ament_prefix if ament_prefix else '(설정되지 않음)'}")
    print(f"  * PYTHONPATH       : {pythonpath if pythonpath else '(설정되지 않음)'}")

    passed = True
    if distro == "humble":
        print(f"  {Colors.GREEN}✓ ROS 2 Distro ('humble') 확인 [PASS]{Colors.RESET}")
    else:
        print(f"  {Colors.RED}✗ ROS_DISTRO가 'humble'이 아닙니다. 'source /opt/ros/humble/setup.bash'를 실행하세요. [FAIL]{Colors.RESET}")
        passed = False

    if "/opt/ros/humble" in ament_prefix:
        print(f"  {Colors.GREEN}✓ AMENT_PREFIX_PATH에 /opt/ros/humble 포함 확인 [PASS]{Colors.RESET}")
    else:
        print(f"  {Colors.RED}✗ AMENT_PREFIX_PATH에 /opt/ros/humble이 누락되었습니다. [FAIL]{Colors.RESET}")
        passed = False

    return passed

def check_cxxabi_symbols():
    print(f"\n{Colors.BOLD}[CHECK 3] CXXABI & libstdc++ Symbol Compatibility{Colors.RESET}")
    passed = True
    required_symbol = "GLIBCXX_3.4.30"

    # 1. System libstdc++
    sys_lib = "/usr/lib/x86_64-linux-gnu/libstdc++.so.6"
    if os.path.exists(sys_lib):
        try:
            out = subprocess.check_output(f"strings {sys_lib} | grep '{required_symbol}'", shell=True).decode()
            if required_symbol in out:
                print(f"  {Colors.GREEN}✓ 시스템 libstdc++: {required_symbol} 지원 확인 [PASS]{Colors.RESET}")
        except Exception:
            print(f"  {Colors.RED}✗ 시스템 libstdc++에서 {required_symbol}을 찾을 수 없습니다. [FAIL]{Colors.RESET}")
            passed = False
    else:
        print(f"  {Colors.YELLOW}⚠ 시스템 라이브러리 경로({sys_lib})를 찾을 수 없습니다.{Colors.RESET}")

    # 2. Conda libstdc++
    conda_prefix = os.environ.get("CONDA_PREFIX", "")
    conda_lib = os.path.join(conda_prefix, "lib", "libstdc++.so.6")
    if os.path.exists(conda_lib):
        if os.path.islink(conda_lib):
            target = os.readlink(conda_lib)
            print(f"  {Colors.GREEN}✓ Conda libstdc++는 시스템 라이브러리로 심볼릭 링크됨: {target} [PASS]{Colors.RESET}")
        else:
            try:
                out = subprocess.check_output(f"strings {conda_lib} | grep '{required_symbol}'", shell=True).decode()
                if required_symbol in out:
                    print(f"  {Colors.GREEN}✓ Conda libstdc++: {required_symbol} 이상 지원 확인 [PASS]{Colors.RESET}")
                else:
                    print(f"  {Colors.RED}✗ Conda libstdc++에 {required_symbol} 심볼이 없습니다! [FAIL]{Colors.RESET}")
                    print(f"    ➔ 조치: 'conda install -c conda-forge libstdcxx-ng -y'를 실행하세요.")
                    passed = False
            except Exception:
                print(f"  {Colors.RED}✗ Conda libstdc++에 {required_symbol} 심볼이 없습니다! [FAIL]{Colors.RESET}")
                print(f"    ➔ 조치: 'conda install -c conda-forge libstdcxx-ng -y'를 실행하세요.")
                passed = False
    else:
        print(f"  {Colors.GREEN}✓ Conda 내 자체 libstdc++ 없음 (시스템 표준 라이브러리 직접 참조 상태) [PASS]{Colors.RESET}")

    return passed

def check_sequential_imports():
    print(f"\n{Colors.BOLD}[CHECK 4] Sequential Heterogeneous Imports (8 Core Modules){Colors.RESET}")
    modules = [
        ("rclpy", "ROS 2 Python Client Library"),
        ("mujoco", "DeepMind Physics Engine"),
        ("open3d", "3D Point Cloud & Vision Library"),
        ("cv2", "OpenCV Computer Vision"),
        ("tf2_ros", "ROS 2 Transform Library"),
        ("py_trees", "Behavior Tree Framework"),
        ("typeguard", "Runtime Type Validation"),
        ("pydot", "Graphviz DOT Graph Visualization"),
    ]

    all_passed = True
    for idx, (mod_name, desc) in enumerate(modules, 1):
        try:
            mod = __import__(mod_name)
            ver = getattr(mod, "__version__", getattr(mod, "version", "loaded"))
            print(f"  {Colors.GREEN}✓ [{idx}/8] {mod_name:<11} (v{ver}) - {desc} [PASS]{Colors.RESET}")
        except Exception as e:
            print(f"  {Colors.RED}✗ [{idx}/8] {mod_name:<11} 로드 실패: {e} [FAIL]{Colors.RESET}")
            all_passed = False

    return all_passed

def check_mujoco_offscreen_rendering():
    print(f"\n{Colors.BOLD}[CHECK 5] MuJoCo Headless & Offscreen RGB-D Rendering{Colors.RESET}")
    try:
        import mujoco
        import numpy as np

        xml = """
        <mujoco model="test_scene">
            <visual>
                <global offwidth="320" offheight="240"/>
            </visual>
            <worldbody>
                <light diffuse="1 1 1" pos="0 0 3" dir="0 0 -1"/>
                <geom name="ground" type="plane" size="1 1 0.1" rgba="0.8 0.8 0.8 1"/>
                <camera name="test_cam" pos="0 -1 0.8" xyaxes="1 0 0 0 0.6 0.8"/>
                <geom name="bottle" type="cylinder" size="0.04 0.12" pos="0 0 0.12" rgba="0.1 0.7 0.2 1"/>
            </worldbody>
        </mujoco>
        """
        model = mujoco.MjModel.from_xml_string(xml)
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)

        renderer = mujoco.Renderer(model, width=320, height=240)
        renderer.update_scene(data, camera="test_cam")
        rgb = renderer.render()

        renderer.enable_depth_rendering()
        renderer.update_scene(data, camera="test_cam")
        depth = renderer.render()
        renderer.disable_depth_rendering()

        if rgb.shape == (240, 320, 3) and depth.shape == (240, 320):
            print(f"  {Colors.GREEN}✓ RGB 버퍼 생성 성공: shape={rgb.shape}, dtype={rgb.dtype} [PASS]{Colors.RESET}")
            print(f"  {Colors.GREEN}✓ Depth 버퍼 생성 성공: shape={depth.shape}, min={depth.min():.2f}m, max={depth.max():.2f}m [PASS]{Colors.RESET}")
            print(f"  {Colors.GREEN}✓ OpenGL 헤드리스 렌더링 컨텍스트 정상 작동 확인 [PASS]{Colors.RESET}")
            return True
        else:
            print(f"  {Colors.RED}✗ 렌더링 버퍼 크기 불일치: RGB {rgb.shape}, Depth {depth.shape} [FAIL]{Colors.RESET}")
            return False
    except Exception as e:
        print(f"  {Colors.RED}✗ MuJoCo 오프스크린 렌더링 실패: {e} [FAIL]{Colors.RESET}")
        print(f"    ➔ 조치: OpenGL/Mesa 헤드리스 라이브러리(libosmesa6-dev, libgl1-mesa-dev) 설치 확인")
        return False

def main():
    print_header("Phase 00: Environment & CXXABI Runtime Binding Diagnostic Tool")
    
    results = [
        ("Python Runtime", check_python_runtime()),
        ("ROS 2 Environment", check_ros2_env()),
        ("CXXABI & libstdc++ Compatibility", check_cxxabi_symbols()),
        ("Sequential Module Imports", check_sequential_imports()),
        ("MuJoCo Offscreen RGB-D Rendering", check_mujoco_offscreen_rendering()),
    ]

    print_header("Diagnostic Summary")
    all_ok = True
    for name, passed in results:
        status = f"{Colors.GREEN}[PASS]{Colors.RESET}" if passed else f"{Colors.RED}[FAIL]{Colors.RESET}"
        print(f"  * {name:<35}: {status}")
        if not passed:
            all_ok = False

    print("\n" + "=" * 60)
    if all_ok:
        print(f"{Colors.BOLD}{Colors.GREEN}🎉 [SUCCESS] 모든 환경 진단 테스트가 성공적으로 통과되었습니다!{Colors.RESET}")
        print(f"{Colors.BOLD}{Colors.GREEN}   Phase 01(기능별 단위 샌드박스 검증)로 진행할 준비가 완료되었습니다.{Colors.RESET}")
        print("=" * 60 + "\n")
        return 0
    else:
        print(f"{Colors.BOLD}{Colors.RED}❌ [FAILED] 일부 검사항목이 실패하였습니다. 위 트러블슈팅 안내를 확인하세요.{Colors.RESET}")
        print("=" * 60 + "\n")
        return 1

if __name__ == "__main__":
    sys.exit(main())
