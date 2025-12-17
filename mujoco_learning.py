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
# print(f"控制范围: motor1={model.actuator_ctrlrange[0]}, motor2={model.actuator_ctrlrange[1]}")

# PD 控制增益（按关节顺序：joint1, joint2）
Kp = np.array([20.0, 20.0])
Kd = np.array([1.0, 1.0])

# 启动可视化
with mujoco.viewer.launch_passive(model, data) as viewer:
    step_count = 0
    index = 0;
    while viewer.is_running():
        # 期望关节角（这里示例为缓慢变化的目标轨迹，你也可以改成常数目标角度）
        q_des =[(0.0, 0.0), (np.pi/2, np.pi/2), (np.pi, np.pi)]

        # 当前关节角和角速度
        q = data.qpos[:2]
        dq = data.qvel[:2]

        # PD 控制律：tau = Kp*(q_des - q) - Kd*dq
        tau = Kp * (q_des[index] - q) - Kd * dq

        # 按执行器控制范围进行饱和
        ctrl_min = model.actuator_ctrlrange[:, 0]
        ctrl_max = model.actuator_ctrlrange[:, 1]
        data.ctrl[:] = np.clip(tau, ctrl_min, ctrl_max)
                
        # 每1000步打印一次关节状态
        if step_count % 10 == 0:
            print(f"时间: {data.time:.2f}s, 关节角度: [{data.qpos[0]:.3f}, {data.qpos[1]:.3f}], "
                  f"控制输入: [{data.ctrl[0]:.2f}, {data.ctrl[1]:.2f}]")
            # print(f"期望关节角: {q_des[index]}", f"当前关节角: ", q)
            
        if step_count % 1000 == 0:
            print(f"期望关节角: {q_des[index]}", f"当前关节角: ", q)
            index += 1
            if index >= len(q_des):
                index = 0
        
        # 仿真步进
        mujoco.mj_step(model, data)
        
        # 同步可视化（限制帧率）
        viewer.sync()
        time.sleep(0.002)  # 匹配仿真时间步长
        
        step_count += 1
        