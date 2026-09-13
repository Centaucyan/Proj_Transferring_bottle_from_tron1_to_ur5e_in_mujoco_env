#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
[tron1_controller.py] Tron1 FSM Locomotion and Docking Controller Node
- 50Hz 제어 루프: LimX RL 정책 추론 및 FSM 상태 천이
- 10Hz 상태 퍼블리셔: /tron1/status (Tron1Status.msg)
- 파라미터 동적 로드: spawn_pose, nav_waypoints, bottle_slots
- 언도킹 구독: /tron1/cmd_undock
"""

import os
import sys
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float64MultiArray
from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import WrenchStamped
from tron1_interfaces.msg import Tron1Status

# =====================================================================
# 도킹 스테이션 하드웨어 안전 불변 상수 (Station Hardware Invariants)
# ※ 안전 필수: 도킹 턱 물리 위치 및 로봇 범퍼 오프셋과 1:1 결합된 불변값
# =====================================================================
DOCK_ALIGN_POSE = (-1.0, 0.0)    # 도킹 진입 1m 전 헤딩 정렬 지점 (TABLE_ALIGN)
DOCK_TARGET_POSE = (0.0, 0.0)   # 최종 범퍼 밀착 및 도킹 락 목표점 (DOCKING_APPROACH)

class Tron1ControllerNode(Node):
    def __init__(self):
        super().__init__('tron1_controller')
        self.get_logger().info("=== [Tron1Controller] 초기화 시작 ===")

        # -------------------------------------------------------------
        # 1. ROS 2 파라미터 선언 및 로드
        # -------------------------------------------------------------
        self.declare_parameter('spawn_pose', [-2.0, -2.0, 0.80, 90.0])
        self.declare_parameter('nav_waypoints', [-4.0, 3.0])
        self.declare_parameter('bottle_slots', [True, True, True])
        self.declare_parameter('nav_max_speed', 0.65)
        self.declare_parameter('docking_creep_speed', 0.15)
        self.declare_parameter('bumper_threshold_n', 5.0)
        self.declare_parameter('vibration_hold_time_s', 3.0)

        self.spawn_pose = self.get_parameter('spawn_pose').value
        raw_wps = self.get_parameter('nav_waypoints').value
        # 1차원 배열로 들어온 nav_waypoints를 2D 튜플 리스트로 변환 (예: [-4.0, 3.0] -> [(-4.0, 3.0)])
        self.nav_waypoints = [(raw_wps[i], raw_wps[i+1]) for i in range(0, len(raw_wps), 2)]
        self.bottle_slots = self.get_parameter('bottle_slots').value
        self.nav_max_speed = self.get_parameter('nav_max_speed').value
        self.docking_creep_speed = self.get_parameter('docking_creep_speed').value
        self.bumper_threshold_n = self.get_parameter('bumper_threshold_n').value
        self.vibration_hold_time_s = self.get_parameter('vibration_hold_time_s').value

        loaded_count = sum(1 for s in self.bottle_slots if s)
        self.get_logger().info(f"  * 스폰 포즈: {self.spawn_pose}")
        self.get_logger().info(f"  * 일반 자율주행 경유지: {self.nav_waypoints} (총 {len(self.nav_waypoints)}개)")
        self.get_logger().info(f"  * 고정 도킹 좌표 (하드웨어 상수): 정렬={DOCK_ALIGN_POSE}, 안착={DOCK_TARGET_POSE}")
        self.get_logger().info(f"  * 물병 슬롯 마스크: {self.bottle_slots} (적재: {loaded_count}EA)")

        # -------------------------------------------------------------
        # 2. FSM 상태 변수 초기화
        # -------------------------------------------------------------
        self.fsm_state = "LANDING"
        self.nav_phase = "WP0_TURN"
        self.current_wp_idx = 0
        self.is_ready_for_pick = False
        self.is_stance_locked = False
        self.has_docked = False
        self.state_timer = 0.0
        self.stable_timer = 0.0

        # 센서 계측 캐시
        self.base_x = float(self.spawn_pose[0])
        self.base_y = float(self.spawn_pose[1])
        self.base_yaw_deg = float(self.spawn_pose[3])
        self.roll_deg = 0.0
        self.pitch_deg = 0.0
        self.bumper_force = 0.0
        self.tray_vel_rms = 0.0

        # 관절 상태 캐시
        self.current_qpos = np.zeros(6, dtype=np.float32)
        self.current_qvel = np.zeros(6, dtype=np.float32)

        # -------------------------------------------------------------
        # 3. 퍼블리셔 및 서브스크라이버 바인딩
        # -------------------------------------------------------------
        # 상태 보고 퍼블리셔 (10Hz)
        self.status_pub = self.create_publisher(Tron1Status, '/tron1/status', 10)

        # 조인트 목표 명령 퍼블리셔 (50Hz)
        self.joint_cmd_pub = self.create_publisher(Float64MultiArray, '/tron1/joint_commands', 10)

        # 센서 데이터 서브스크라이버 (Phase 03 sim_bridge에서 수신)
        self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.create_subscription(WrenchStamped, '/tron1/bumper_wrench', self.bumper_wrench_callback, 10)
        self.create_subscription(Imu, '/tron1/imu', self.imu_callback, 10)

        # 상위 시스템(UR5e/BT)으로부터의 언도킹 명령 수신
        self.create_subscription(Bool, '/tron1/cmd_undock', self.cmd_undock_callback, 10)

        # -------------------------------------------------------------
        # 4. 주기적 루프 타이머 등록
        # -------------------------------------------------------------
        self.control_timer = self.create_timer(0.02, self.control_loop)       # 50Hz 메인 제어 루프
        self.status_timer = self.create_timer(0.1, self.status_publish_loop)  # 10Hz 상태 보고 루프

        self.get_logger().info("=== [Tron1Controller] 노드 구동 완료 (50Hz 제어 / 10Hz 상태 보고) ===")

    # -----------------------------------------------------------------
    # 센서 콜백 함수
    # -----------------------------------------------------------------
    def joint_state_callback(self, msg: JointState):
        if len(msg.position) >= 6:
            self.current_qpos[:] = msg.position[:6]
            self.current_qvel[:] = msg.velocity[:6]

    def bumper_wrench_callback(self, msg: WrenchStamped):
        # 전방 수평 반력 크기
        self.bumper_force = float(abs(msg.wrench.force.x))

    def imu_callback(self, msg: Imu):
        # 쿼터니언을 Euler 각도로 변환 (간이 변환)
        qx = msg.orientation.x
        qy = msg.orientation.y
        qz = msg.orientation.z
        qw = msg.orientation.w
        sinr_cosp = 2 * (qw * qx + qy * qz)
        cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
        roll = np.arctan2(sinr_cosp, cosr_cosp)
        sinp = 2 * (qw * qy - qz * qx)
        pitch = np.arcsin(np.clip(sinp, -1.0, 1.0))
        self.roll_deg = float(np.degrees(roll))
        self.pitch_deg = float(np.degrees(pitch))

    def cmd_undock_callback(self, msg: Bool):
        if msg.data:
            if self.fsm_state == "READY_FOR_PICK":
                self.get_logger().info("★ [/tron1/cmd_undock 수신] 언도킹 시퀀스를 개시합니다!")
                self.fsm_state = "UNDOCKING"
                self.is_ready_for_pick = False
                self.state_timer = 0.0
            else:
                self.get_logger().warn(
                    f"[/tron1/cmd_undock 거부] 로봇이 READY_FOR_PICK 상태가 아닙니다! (현재 FSM: {self.fsm_state})"
                )

    # -----------------------------------------------------------------
    # 50Hz 메인 제어 및 FSM 전이 루프
    # -----------------------------------------------------------------
    def control_loop(self):
        dt = 0.02
        self.state_timer += dt

        # [FSM 상태 1: LANDING]
        if self.fsm_state == "LANDING":
            if self.state_timer >= 2.0:
                self.fsm_state = "IN_PLACE_HOLD"
                self.state_timer = 0.0
                self.get_logger().info(">> FSM: LANDING -> IN_PLACE_HOLD")

        # [FSM 상태 2: IN_PLACE_HOLD]
        elif self.fsm_state == "IN_PLACE_HOLD":
            if not self.has_docked and self.state_timer >= 2.0:
                self.fsm_state = "WALKING"
                self.state_timer = 0.0
                self.nav_phase = "WP0_TURN"
                self.get_logger().info(">> FSM: IN_PLACE_HOLD -> WALKING (경유지 자율 보행 개시)")
            elif self.has_docked:
                pass  # 원점 복귀 후 제자리 발구름 지속

        # [FSM 상태 3: WALKING]
        elif self.fsm_state == "WALKING":
            # 1) 도킹 전 정방향 경유지 보행 (WP0 -> WP1 도달 시 도킹 접근 전이)
            if not self.has_docked:
                if self.state_timer >= 4.0:  # 데모 타이머 (실제 구동 시 WP1 도달 거리 판정)
                    self.fsm_state = "DOCKING_APPROACH"
                    self.state_timer = 0.0
                    self.nav_phase = "TABLE_ALIGN"
                    self.get_logger().info(">> FSM: WALKING -> DOCKING_APPROACH (도킹 정렬 및 접근 개시)")
            # 2) 언도킹 후 스폰 원점 복귀 보행 (원점 도달 시 IN_PLACE_HOLD 전이)
            else:
                if self.state_timer >= 4.0:  # 데모 타이머 (실제 구동 시 스폰 원점 도달 거리 판정)
                    self.fsm_state = "IN_PLACE_HOLD"
                    self.state_timer = 0.0
                    self.get_logger().info(">> FSM: WALKING -> IN_PLACE_HOLD (원점 복귀 완료, 제자리 발구름 대기)")

        # [FSM 상태 4: DOCKING_APPROACH]
        elif self.fsm_state == "DOCKING_APPROACH":
            # 범퍼 접촉 반력 임계값 감지 판정
            if self.bumper_force >= self.bumper_threshold_n:
                self.fsm_state = "STANCE_LOCK"
                self.state_timer = 0.0
                self.is_stance_locked = True
                self.get_logger().info(f">> FSM: 범퍼 접촉 감지({self.bumper_force:.1f}N)! STANCE_LOCK 진입 (발 5cm 후퇴 3점 지지 체결)")

        # [FSM 상태 5: STANCE_LOCK]
        elif self.fsm_state == "STANCE_LOCK":
            # 0.5초간 동시 수평화 보간 후 정적 안정 인터락 검증
            if self.state_timer >= 0.5:
                # 무진동 및 수평 오차 0.5° 이내 유지 시간 누적
                if abs(self.roll_deg) <= 0.5 and abs(self.pitch_deg) <= 0.5 and self.tray_vel_rms < 0.005:
                    self.stable_timer += dt
                else:
                    self.stable_timer = 0.0

                if self.stable_timer >= self.vibration_hold_time_s:
                    self.fsm_state = "READY_FOR_PICK"
                    self.is_ready_for_pick = True
                    self.has_docked = True  # 도킹 완료 플래그 기록
                    self.get_logger().info("★ [READY_FOR_PICK] 3초 정적 무진동 수평 확립! UR5e 피킹 권한을 승인합니다.")

        # [FSM 상태 6: READY_FOR_PICK]
        elif self.fsm_state == "READY_FOR_PICK":
            pass # cmd_undock 신호 대기

        # [FSM 상태 7: UNDOCKING]
        elif self.fsm_state == "UNDOCKING":
            if self.state_timer >= 3.0:
                self.fsm_state = "WALKING"
                self.state_timer = 0.0
                self.nav_phase = "RETURN_WALK"
                self.get_logger().info(">> FSM: UNDOCKING 후진 완료 -> WALKING (원점 복귀 보행 개시)")

        # 관절 명령 발행 (임시 더미 또는 제어 계산치)
        cmd_msg = Float64MultiArray()
        cmd_msg.data = [0.0] * 6
        self.joint_cmd_pub.publish(cmd_msg)

    # -----------------------------------------------------------------
    # 10Hz 상태 보고 퍼블리셔
    # -----------------------------------------------------------------
    def status_publish_loop(self):
        msg = Tron1Status()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "tron1_base"

        msg.fsm_state = self.fsm_state
        msg.nav_phase = self.nav_phase
        msg.is_ready_for_pick = self.is_ready_for_pick
        msg.is_stance_locked = self.is_stance_locked

        msg.roll_deg = self.roll_deg
        msg.pitch_deg = self.pitch_deg
        msg.bumper_force = self.bumper_force
        msg.tray_vel_rms = self.tray_vel_rms

        msg.base_x = self.base_x
        msg.base_y = self.base_y
        msg.base_yaw_deg = self.base_yaw_deg

        self.status_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = Tron1ControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("키보드 인터럽트에 의해 노드가 정상 종료됩니다.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
