import torch

# 直接加载你的模型，不初始化网络（最简单、最稳的方式）
ckpt = torch.load("/root/unitree_rl_lab/logs/rsl_rl/unitree_g1_23dof_velocity/test1/model_4999.pt", map_location="cpu")
# 提取策略网络
actor = ckpt["model_state_dict"]
# 导出模型
torch.save(actor, "/root/unitree_mujoco/simulate_python/g1_23dof_policy.pt")

print("✅ 模型导出成功！已放到仿真文件夹！")