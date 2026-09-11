import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.font_manager as fm

# Set Korean Font
font_path = '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
prop = fm.FontProperties(fname=font_path)
prop_bold = fm.FontProperties(fname='/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc')

fig, ax = plt.subplots(figsize=(15, 11), dpi=200)
ax.set_facecolor('#F8F9FA')
fig.patch.set_facecolor('#F8F9FA')

# X coordinates for swimlanes
lanes = {
    'Tron1': 2.0,
    'ROS2': 6.0,
    'Vision': 10.0,
    'UR5e': 14.0
}

lane_colors = {
    'Tron1': '#1E88E5',
    'ROS2': '#43A047',
    'Vision': '#8E24AA',
    'UR5e': '#E53935'
}

# Draw Header Boxes
for name, x in lanes.items():
    color = lane_colors[name]
    rect = patches.FancyBboxPatch((x - 1.4, 18.2), 2.8, 0.9, boxstyle="round,pad=0.1",
                                  ec=color, fc=color, zorder=3)
    ax.add_patch(rect)
    
    sub = {
        'Tron1': 'Tron1 이족보행 AMR\n(tron1_controller)',
        'ROS2': 'ROS 2 통신 버스\n(Topics / Actions)',
        'Vision': 'Eye-in-Hand D435i\n(bottle_detector_3d)',
        'UR5e': 'UR5e + MoveIt 2\n(ur5e_pick_place)'
    }[name]
    
    ax.text(x, 18.65, sub, ha='center', va='center', color='white', 
            fontproperties=prop_bold, fontsize=10.5, zorder=4)

# Draw Lifelines
for name, x in lanes.items():
    ax.plot([x, x], [18.2, 0.5], color='#B0BEC5', linestyle='--', linewidth=1.5, zorder=1)

# Helper for drawing message arrow
def draw_msg(y, x_start, x_end, text, subtext="", color='#1A237E', is_topic=True):
    dx = x_end - x_start
    ax.annotate("", xy=(x_end, y), xytext=(x_start, y),
                arrowprops=dict(arrowstyle="->", color=color, lw=2.0, mutation_scale=15),
                zorder=3)
    mid_x = (x_start + x_end) / 2
    tag = "[Topic]" if is_topic else "[Action/Msg]"
    ax.text(mid_x, y + 0.22, f"{text}", ha='center', va='bottom', color=color,
            fontproperties=prop_bold, fontsize=9.5, zorder=4,
            bbox=dict(boxstyle="round,pad=0.2", fc='white', ec=color, lw=1.2, alpha=0.95))
    if subtext:
        ax.text(mid_x, y - 0.28, subtext, ha='center', va='top', color='#455A64',
                fontproperties=prop, fontsize=8.5, zorder=4)

# Helper for action note / state box
def draw_state(lane, y, text, height=0.6, width=2.4, color='#ECEFF1', border_color='#607D8B'):
    x = lanes[lane]
    rect = patches.FancyBboxPatch((x - width/2, y - height/2), width, height,
                                  boxstyle="round,pad=0.08", ec=border_color, fc=color, lw=1.2, zorder=3)
    ax.add_patch(rect)
    ax.text(x, y, text, ha='center', va='center', color='#263238',
            fontproperties=prop, fontsize=8.5, zorder=4)

# Helper for phase separator block
def draw_phase(y, title):
    rect = patches.FancyBboxPatch((0.2, y - 0.25), 15.6, 0.5,
                                  boxstyle="round,pad=0.05", ec='#CFD8DC', fc='#ECEFF1', lw=1, alpha=0.8, zorder=2)
    ax.add_patch(rect)
    ax.text(0.5, y, title, ha='left', va='center', color='#37474F',
            fontproperties=prop_bold, fontsize=10.5, zorder=4)

# --- Sequence Steps ---

# Phase 1
draw_phase(17.3, "Phase 1: Tron1 자율 보행 및 정밀 도킹 (Autonomous Navigation & Stance Lock)")
draw_state('Tron1', 16.3, "웨이포인트 보행 이동\n(WP0 → WP1 → 정면 정렬)", height=0.6)
draw_state('Tron1', 15.3, "극저속 도킹 크리핑 (0.04m/s)\n범퍼 접촉(F ≥ 5N) 감지", height=0.6)
draw_state('Tron1', 14.3, "발 5cm 후퇴 3점 지지 형성\n발구름 정지 (Stance Lock)", height=0.6)
draw_state('Tron1', 13.3, "고관절 신전 동시 수평화 (+0.18rad)\nRoll = -0.02°, Pitch = +0.13°", height=0.6)
draw_state('Tron1', 12.3, "3.0s 무진동 수평 인터락 통과\n도킹 클램프 체결 (0.000mm 부동)", height=0.6)

# Phase 2
draw_phase(11.4, "Phase 2: 도킹 완료 핸드셰이크 & 상태 브로드캐스트 (Interlock Handshake)")
draw_msg(10.6, lanes['Tron1'], lanes['ROS2'], "/tron1/status", "READY_FOR_PICK (True)", color='#1E88E5')
draw_msg(9.8, lanes['ROS2'], lanes['Vision'], "/tron1/status", "READY_FOR_PICK 수신 → 스캔 트리거", color='#1E88E5')
draw_msg(9.0, lanes['ROS2'], lanes['UR5e'], "/tron1/status", "도킹 완료 인지 → 모션 준비", color='#1E88E5')

# Phase 3
draw_phase(8.1, "Phase 3: Eye-in-Hand 3D 비전 인식 & TF2 좌표 변환 (Perception Pipeline)")
draw_state('Vision', 7.2, "D435i RGB-D 영상 취득\n(Scan Pose 트레이 하향 조준)", height=0.55)
draw_state('Vision', 6.3, "OpenCV & Open3D 처리\nRANSAC 평면 제거 + 물병 3D 중심점", height=0.55)
draw_msg(5.4, lanes['Vision'], lanes['ROS2'], "/bottle/centroid_3d", "PointStamped (x, y, z) + TF2 변환", color='#8E24AA')
draw_msg(4.7, lanes['ROS2'], lanes['UR5e'], "/bottle/centroid_3d", "파지 목표 좌표 전달", color='#8E24AA')

# Phase 4
draw_phase(3.9, "Phase 4: MoveIt 2 충돌 회피 Pick & Place 반복 (Manipulation Loop)")
draw_state('UR5e', 3.1, "Approach(상공 접근) → Grasp(파지)\n→ Lift(수직 10cm) → Place(테이블 안착)", height=0.55)
draw_state('UR5e', 2.3, "Scan Pose 복귀 및 물병 3개 반복\n(전체 물병 이송 완료 판정)", height=0.55)

# Phase 5
draw_phase(1.5, "Phase 5: 작업 완료 및 안전 언도킹 복귀 (Task Completion & Undocking)")
draw_msg(0.9, lanes['UR5e'], lanes['ROS2'], "/tron1/cmd_undock", "ALL_BOTTLES_TRANSFERRED (True)", color='#E53935')
draw_msg(0.4, lanes['ROS2'], lanes['Tron1'], "/tron1/cmd_undock", "Stance Lock 해제 → 0.15m 후진 언도킹 → 복귀 보행", color='#E53935')

ax.set_xlim(0.0, 16.0)
ax.set_ylim(0.0, 19.5)
ax.axis('off')

plt.tight_layout()
plt.savefig('documents/scenario/ros2_communication_flow.png', dpi=200, bbox_inches='tight')
print("Successfully generated documents/scenario/ros2_communication_flow.png")
