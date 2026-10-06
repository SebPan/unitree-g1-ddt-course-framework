import pandas as pd
import numpy as np

# ===================== 【你原生23DOF正确配置】=====================
# 原始29DOF CSV路径
INPUT_CSV = "/root/unitree_rl_lab/deploy/robots/g1_23dof/config/policy/mimic/dance_102/params/G1_Take_102.bvh_60hz.csv"
# 输出覆盖路径
OUTPUT_CSV = "/root/unitree_rl_lab/deploy/robots/g1_23dof/config/policy/mimic/dance_102/params/G1_Take_102.bvh_60hz.csv"

# 【核心】你原生23DOF关节索引（和csv_to_npz23完全一致，绝对不越界）
SELECTED_JOINTS = [
    0,1,2,3,4,5,        # 左腿
    6,7,8,9,10,11,      # 右腿
    12,                 # 腰部 (仅1个，23DOF专属)
    13,14,15,16,17,     # 左臂
    18,19,20,21,22      # 右臂
]
# =================================================================

# 读取原始29DOF数据
df = pd.read_csv(INPUT_CSV)
root_data = df.iloc[:, :7]          # 根状态 (7列)
joint_pos = df.iloc[:, 7:36]        # 29个关节位置
joint_vel = df.iloc[:, 36:65]       # 29个关节速度

# 裁剪为23DOF
joint_pos_23 = joint_pos.iloc[:, SELECTED_JOINTS]
joint_vel_23 = joint_vel.iloc[:, SELECTED_JOINTS]

# 保存新CSV
final_df = pd.concat([root_data, joint_pos_23, joint_vel_23], axis=1)
final_df.to_csv(OUTPUT_CSV, index=False)

print("✅ 23DOF专用CSV生成成功！已覆盖原文件！")
print(f"📊 关节数：29 → 23 | 无越界、无报错")