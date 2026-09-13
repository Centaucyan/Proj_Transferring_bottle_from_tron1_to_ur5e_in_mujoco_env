# [Tron1Status.msg] Tron1 로봇 상태 및 도킹 인터락 모니터링 메시지
# 최종 수정일: 2026-09-11

std_msgs/Header header

# 1. 유한 상태 머신 (FSM) 상태
# 가능 상태: LANDING, IN_PLACE_HOLD, WALKING, DOCKING_APPROACH, STANCE_LOCK, READY_FOR_PICK, UNDOCKING
string fsm_state

# 2. 내비게이션 세부 페이즈 (DOCKING_APPROACH 상태일 때)
# 가능 페이즈: WP0_TURN, WP0_APPROACH, WP1_TURN, WP1_APPROACH, WP2_CREEP_DOCK
string nav_phase

# 3. 상위 협업(UR5e 로봇팔) 인터락 플래그
bool is_ready_for_pick      # 발 5cm 후퇴 + 동시 수평화 + 3초 안정 판정 시 True
bool is_stance_locked       # 발구름 정지 및 3점 지지 정적 인터락 체결 시 True

# 4. 실시간 자세 및 물리 센서 측정값
float32 roll_deg            # 횡방향 수평 오차각 (도 단위, 정상 범위: -0.5° ~ +0.5°)
float32 pitch_deg           # 종방향 수평 오차각 (도 단위, 정상 범위: -0.5° ~ +0.5°)
float32 bumper_force        # 전면 범퍼 터치 접촉 반력 (N 단위)
float32 tray_vel_rms        # 트레이 진동 속도 RMS (m/s 단위, 무진동 판정: < 0.005)

# 5. 현재 로봇 베이스 글로벌 2D 위치 (Odometry 추정치)
float32 base_x
float32 base_y
float32 base_yaw_deg
