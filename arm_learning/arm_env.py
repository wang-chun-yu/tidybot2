import mujoco
import mujoco.viewer
import numpy as np
import time

# 加载模型
model = mujoco.MjModel.from_xml_path('arm_models/scene.xml')
data = mujoco.MjData(model)

print(f"自由度数量: {model.nv}")
print(f"关节数量: {model.njnt}")
print(f"执行器数量: {model.nu}")

# 关节数
joint_num = 3
# PD 控制增益（按关节顺序：joint1, joint2）
Kp = np.array([100.0, 50.0, 50.0])
Kd = np.array([1.0, 1.0, 1.0])

def as_hmat(xpos, xmat_flat):
    R = np.array(xmat_flat).reshape(3, 3)   # 行主序
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3]  = xpos
    return T

def R_to_quat(R):
    quat = np.zeros(4)
    quat[0] = np.sqrt(1 + R[0, 0] + R[1, 1] + R[2, 2]) / 2
    quat[1] = (R[2, 1] - R[1, 2]) / (4 * quat[0])
    quat[2] = (R[0, 2] - R[2, 0]) / (4 * quat[0])
    quat[3] = (R[1, 0] - R[0, 1]) / (4 * quat[0])
    return quat

def R_to_euler(R):
    euler = np.zeros(3)
    euler[0] = np.arctan2(2 * (R[2, 1] * R[2, 2] + R[0, 0]), 1 - 2 * (R[1, 1] + R[2, 2]))
    euler[1] = np.arcsin(2 * (R[2, 0] - R[0, 2]))
    euler[2] = np.arctan2(2 * (R[1, 0] * R[1, 1] + R[2, 2]), 1 - 2 * (R[1, 1] + R[2, 2]))
    return euler

# model.opt.gravity = np.array([0.0, 0.0, 0.0])
print("model.body_gravcomp: ", model.body_gravcomp)
model.body_gravcomp[:] = 1.0
body_names = {model.body(i).name for i in range(model.nbody)}
print("body_names: ", body_names)
for object_name in ['cube', 'world']:
    if object_name in body_names:
        model.body_gravcomp[model.body(object_name).id] = 0.0
print("model.body_gravcomp after: ", model.body_gravcomp)

# 启动可视化
with mujoco.viewer.launch_passive(model, data) as viewer:
    step_count = 0
    index = 0;
    # 以第0个 body、geom 为例
    body_id = 1
    geom_id = 1

    # T_body = as_hmat(data.xpos[body_id], data.xmat[body_id])
    # T_geom = as_hmat(data.geom_xpos[geom_id], data.geom_xmat[geom_id])
    # print("T_body: ", T_body)
    # print("T_geom: ", T_geom)
    # print("data.geom_xpos: ", data.geom_xpos[geom_id])
    # print("data.geom_xmat: ", data.geom_xmat[geom_id])
    # print("R_to_quat(data.geom_xmat[geom_id]): ", R_to_quat(data.geom_xmat[geom_id].reshape(3, 3)))
    # print("R_to_euler(data.geom_xmat[geom_id]): ", R_to_euler(data.geom_xmat[geom_id].reshape(3, 3)))
    while viewer.is_running():
        # 期望关节角（这里示例为缓慢变化的目标轨迹，你也可以改成常数目标角度）
        q_des =[(0.0, 0.0, 0.0), (np.pi/2, -np.pi/2, -np.pi/2), (np.pi, -np.pi, np.pi/2)]
        # 当前关节角和角速度
        q = data.qpos[:joint_num]
        dq = data.qvel[:joint_num]

        # PD 控制律：tau = Kp*(q_des - q) - Kd*dq
        tau = Kp * (q_des[index] - q) - Kd * dq

        # 按执行器控制范围进行饱和
        ctrl_min = model.actuator_ctrlrange[:, 0]
        ctrl_max = model.actuator_ctrlrange[:, 1]
        data.ctrl[:joint_num] = np.clip(tau, ctrl_min, ctrl_max)
                
        # 每1000步打印一次关节状态
        if step_count % 10 == 0:
            for i in range(joint_num):
                print(f"关节{i+1}角度: {data.qpos[i]:.3f}, 控制输入: {data.ctrl[i]:.2f}")
            
        if step_count % 1000 == 0:
            for i in range(joint_num):
                print(f"关节{i+1}期望角度: {q_des[index][i]:.3f}, 当前角度: {q[i]:.3f}")
            if step_count % 3000 == 0:
                index += 1
                if index >= len(q_des):
                    index = 0
        
        # 仿真步进
        # time.sleep(100000)  # 匹配仿真时间步长
        mujoco.mj_step(model, data)
        
        # 同步可视化（限制帧率）
        viewer.sync()
        # time.sleep(0.002)  # 匹配仿真时间步长
        
        step_count += 1
        