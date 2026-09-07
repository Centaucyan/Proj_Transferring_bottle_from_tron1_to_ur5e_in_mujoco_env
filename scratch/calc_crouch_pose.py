import math
import numpy as np

# Tron1 다리 기구학 파라미터
# base to abad_L
p_base_abad = np.array([0.05556, 0.105, -0.2602])
# abad to hip_L
p_abad_hip = np.array([-0.077, 0.0205, 0.0])
# hip to knee_L (L1)
p_hip_knee_0 = np.array([-0.15, -0.0205, -0.25981])
# knee to foot_L (L2)
p_knee_foot_0 = np.array([0.150, 0.0, -0.2598])
foot_radius = 0.032

def rot_y(angle):
    c = math.cos(angle)
    s = math.sin(angle)
    return np.array([
        [c, 0, s],
        [0, 1, 0],
        [-s, 0, c]
    ])

def forward_kinematics(q_hip, q_knee):
    # hip rotation: axis is [0, 1, 0], angle is q_hip
    R_hip = rot_y(q_hip)
    # knee rotation: axis is [0, -1, 0], angle is q_knee -> rotation about Y is -q_knee
    R_knee = rot_y(-q_knee)
    
    # knee position relative to hip
    p_knee_rel_hip = R_hip @ p_hip_knee_0
    
    # foot position relative to hip
    R_knee_total = R_hip @ R_knee
    p_foot_rel_knee = R_knee_total @ p_knee_foot_0
    
    p_foot_rel_hip = p_knee_rel_hip + p_foot_rel_knee
    
    # relative to base
    p_hip_rel_base = p_base_abad + p_abad_hip
    p_knee_rel_base = p_hip_rel_base + p_knee_rel_hip
    p_foot_rel_base = p_hip_rel_base + p_foot_rel_hip
    
    return p_hip_rel_base, p_knee_rel_base, p_foot_rel_base

# Stand pose
hip_stand = 0.40
knee_stand = 0.80
_, p_knee, p_foot = forward_kinematics(hip_stand, knee_stand)
print(f"Stand pose (hip={hip_stand}, knee={knee_stand}):")
print(f"  Foot rel base: {p_foot}, required base Z: {-p_foot[2] + foot_radius:.3f}")
print(f"  Knee rel base: {p_knee}, knee Z above ground: {p_knee[2] - p_foot[2] + foot_radius:.3f}")

print("\nSearching for Folded/Crouch pose:")
# 사진 분석:
# 1. 허벅지가 뒤로 가거나, 무릎이 깊게 굽혀짐.
# 2. 바닥에서 몸통 높이가 낮음 (0.45 ~ 0.55m).
# 3. 무릎 최하단이 바닥에 거의 닿을락말락 (knee Z above ground ~ 0.05 ~ 0.10m).
# 4. 발끝은 앞쪽 바닥에 안착.
for q_h in np.linspace(-0.8, 1.2, 21):
    for q_k in np.linspace(0.5, 1.35, 18):
        _, pk, pf = forward_kinematics(q_h, q_k)
        base_z = -pf[2] + foot_radius
        knee_z_ground = pk[2] + base_z
        # 사진처럼 base_z가 0.45~0.58m 사이이고, 무릎이 바닥 근처(0.04~0.12m)인 경우
        if 0.45 <= base_z <= 0.60 and 0.04 <= knee_z_ground <= 0.12:
            print(f"q_hip={q_h:5.2f}, q_knee={q_k:5.2f} -> Base_Z={base_z:.3f}m, Knee_Z={knee_z_ground:.3f}m, Foot_X={pf[0]:.3f}m")
