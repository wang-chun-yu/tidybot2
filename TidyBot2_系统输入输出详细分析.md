# TidyBot2 机械臂控制系统输入输出详细分析

## 系统总体架构

```
外部输入 → RPC服务器 → 控制器 → 硬件接口 → 机械臂硬件
   ↑                                        ↓
传感器反馈 ← 状态估计 ← 传感器接口 ← 硬件传感器
```

## 1. 系统级输入输出

### 1.1 最终系统输入（用户/策略层面）
```python
# 动作空间定义
action = {
    'base_pose': np.array([x, y, theta]),     # 底盘目标位姿
                                              # x, y: 位置 (m)
                                              # theta: 朝向角度 (rad)
    
    'arm_pos': np.array([x, y, z]),           # 机械臂末端位置 (m)
                                              # 相对于机械臂基座坐标系
    
    'arm_quat': np.array([x, y, z, w]),       # 机械臂末端姿态（四元数）
                                              # 标准四元数格式 (x,y,z,w)
    
    'gripper_pos': np.array([pos]),           # 夹爪开合程度 [0,1]
                                              # 0: 完全闭合, 1: 完全打开
}
```

### 1.2 最终系统输出（观测空间）
```python
# 观测空间定义
obs = {
    # 底盘状态
    'base_pose': np.array([x, y, theta]),     # 当前底盘位姿 (m, m, rad)
    'base_vel': np.array([vx, vy, omega]),    # 底盘速度 (m/s, m/s, rad/s)
    
    # 机械臂状态  
    'arm_pos': np.array([x, y, z]),           # 末端执行器位置 (m)
    'arm_quat': np.array([x, y, z, w]),       # 末端执行器姿态（四元数）
    'gripper_pos': np.array([pos]),           # 夹爪位置 [0,1]
    
    # 视觉信息
    'base_image': np.array(H, W, 3),          # 底盘相机图像 (RGB)
                                              # 形状: (84, 84, 3), dtype: uint8
    'wrist_image': np.array(H, W, 3),         # 腕部相机图像 (RGB)  
                                              # 形状: (84, 84, 3), dtype: uint8
}
```

## 2. RPC服务器层输入输出

### 2.1 arm_server.py (机械臂RPC服务器)

#### 输入接口
```python
# execute_action() 方法输入
action_input = {
    'arm_pos': np.array([x, y, z]),           # 目标末端位置 (m)
    'arm_quat': np.array([x, y, z, w]),       # 目标末端姿态（四元数）
    'gripper_pos': np.array([pos])            # 目标夹爪位置 [0,1]
}
```

#### 输出接口
```python
# get_state() 方法输出
arm_state = {
    'arm_pos': np.array([x, y, z]),           # 当前末端位置 (m)
    'arm_quat': np.array([x, y, z, w]),       # 当前末端姿态（四元数）
    'gripper_pos': np.array([pos])            # 当前夹爪位置 [0,1]
}
```

#### 内部处理流程
1. **接收动作** → `execute_action(action)`
2. **逆运动学求解** → `ik_solver.solve(arm_pos, arm_quat, current_q)`
3. **关节角度输出** → `qpos: np.array(7,)` + `gripper_pos: float`
4. **命令队列** → `command_queue.put((qpos, gripper_pos))`

## 3. 控制器层输入输出

### 3.1 JointCompliantController (关节顺应性控制器)

#### 输入数据流
```python
# 从command_queue接收的命令
command_input = (
    qpos,           # np.array(7,) - 目标关节角度 (rad)
    gripper_pos     # float - 目标夹爪位置 [0,1]
)

# 从机械臂硬件接收的传感器数据
sensor_input = {
    'q': np.array(7,),      # 当前关节角度 (rad)
    'dq': np.array(7,),     # 当前关节速度 (rad/s) 
    'tau': np.array(7,),    # 当前关节力矩 (N⋅m)
    'gripper_pos': float    # 当前夹爪位置 [0,1]
}
```

#### 输出数据流
```python
# control_callback() 返回值
control_output = (
    tau_c,          # np.array(7,) - 控制力矩命令 (N⋅m)
    gripper_pos     # float - 夹爪位置命令 [0,1]
)
```

#### 内部状态变量
```python
# 控制器内部维护的状态
internal_states = {
    # 传感器状态（展开角度）
    'q_s': np.array(7,),        # 传感器关节角度 (rad)
    
    # 期望状态（来自轨迹生成器）
    'q_d': np.array(7,),        # 期望关节角度 (rad)
    'dq_d': np.array(7,),       # 期望关节速度 (rad/s)
    
    # 名义模型状态
    'q_n': np.array(7,),        # 名义关节角度 (rad)
    'dq_n': np.array(7,),       # 名义关节速度 (rad/s)
    
    # 滤波器状态
    'tau_filter': LowPassFilter # 力矩低通滤波器
}
```

### 3.2 在线轨迹生成器 (Ruckig OTG)

#### 输入参数
```python
# OTG输入参数
otg_input = {
    'current_position': np.array(7,),     # 当前关节角度 (rad)
    'current_velocity': np.array(7,),     # 当前关节速度 (rad/s)
    'target_position': np.array(7,),      # 目标关节角度 (rad)
    'target_velocity': np.array(7,),      # 目标关节速度 (rad/s) - 通常为0
    'max_velocity': np.array(7,),         # 最大速度限制 (rad/s)
    'max_acceleration': np.array(7,)      # 最大加速度限制 (rad/s²)
}

# 速度限制配置
max_velocity = [
    math.radians(80),   # 关节1-4: 80°/s
    math.radians(80), 
    math.radians(80), 
    math.radians(80),
    math.radians(140),  # 关节5-7: 140°/s
    math.radians(140),
    math.radians(140)
]

# 加速度限制配置  
max_acceleration = [
    math.radians(240),  # 关节1-4: 240°/s²
    math.radians(240),
    math.radians(240), 
    math.radians(240),
    math.radians(450),  # 关节5-7: 450°/s²
    math.radians(450),
    math.radians(450)
]
```

#### 输出参数
```python
# OTG输出参数
otg_output = {
    'new_position': np.array(7,),         # 下一时刻关节角度 (rad)
    'new_velocity': np.array(7,),         # 下一时刻关节速度 (rad/s)
    'new_acceleration': np.array(7,)      # 下一时刻关节加速度 (rad/s²)
}
```

### 3.3 逆运动学求解器 (IKSolver)

#### 输入参数
```python
# solve() 方法输入
ik_input = {
    'pos': np.array([x, y, z]),           # 目标末端位置 (m)
    'quat': np.array([x, y, z, w]),       # 目标末端姿态（四元数）
    'curr_qpos': np.array(7,),            # 当前关节角度 (rad) - 作为初值
    'max_iters': int,                     # 最大迭代次数 (默认20)
    'err_thresh': float                   # 误差阈值 (默认1e-4)
}
```

#### 输出参数
```python
# solve() 方法输出
ik_output = np.array(7,)                  # 求解的关节角度 (rad)
```

#### 内部计算过程
```python
# 中间计算变量
ik_internal = {
    'err': np.array(6,),                  # 位置+姿态误差
    'err_pos': np.array(3,),              # 位置误差 (m)
    'err_rot': np.array(3,),              # 姿态误差 (rad)
    'jac': np.array((6, 7)),              # 雅可比矩阵
    'jac_pos': np.array((3, 7)),          # 位置雅可比
    'jac_rot': np.array((3, 7)),          # 姿态雅可比
    'dq': np.array(7,)                    # 关节角度增量 (rad)
}
```

## 4. 硬件接口层输入输出

### 4.1 TorqueControlledArm (Kinova硬件接口)

#### 输入命令
```python
# control_callback() 接收的控制命令
hardware_input = (
    tau_c,          # np.array(7,) - 关节力矩命令 (N⋅m)
    gripper_pos     # float - 夹爪位置命令 [0,1]
)
```

#### 传感器反馈
```python
# 硬件传感器反馈
hardware_feedback = {
    'q': np.array(7,),                    # 关节角度 (rad)
    'dq': np.array(7,),                   # 关节速度 (rad/s)
    'tau': np.array(7,),                  # 关节力矩 (N⋅m)
    'gripper_pos': float,                 # 夹爪位置 [0,1]
    'tool_pose': (
        np.array([x, y, z]),              # 末端位置 (m)
        np.array([x, y, z, w])            # 末端姿态（四元数）
    )
}
```

#### 内部转换过程
```python
# 力矩到电流转换
torque_to_current = {
    'torque_constant': np.array([11.0, 11.0, 11.0, 11.0, 7.6, 7.6, 7.6]),
    'current_limit_max': np.array([10.0, 10.0, 10.0, 10.0, 6.0, 6.0, 6.0]),
    'current_limit_min': np.array([-10.0, -10.0, -10.0, -10.0, -6.0, -6.0, -6.0])
}

# 电流命令计算
current_cmd = np.clip(tau_c / torque_constant, current_limit_min, current_limit_max)
```

## 5. 控制算法中间过程详解

### 5.1 PD控制器计算过程

#### 输入
```python
pd_input = {
    'q_n': np.array(7,),      # 名义模型关节角度 (rad)
    'dq_n': np.array(7,),     # 名义模型关节速度 (rad/s)
    'q_d': np.array(7,),      # 期望关节角度 (rad)  
    'dq_d': np.array(7,),     # 期望关节速度 (rad/s)
    'g': np.array(7,)         # 重力补偿力矩 (N⋅m)
}
```

#### 计算过程
```python
# 误差计算
position_error = q_n - q_d                # 位置误差 (rad)
velocity_error = dq_n - dq_d              # 速度误差 (rad/s)

# PD控制力矩
tau_task = -K_p @ position_error - K_d @ velocity_error + g
```

#### 输出
```python
tau_task = np.array(7,)                   # 任务空间力矩 (N⋅m)
```

### 5.2 名义动力学模型更新

#### 输入
```python
nominal_input = {
    'tau_task': np.array(7,),             # 任务力矩 (N⋅m)
    'tau_s_f': np.array(7,),              # 滤波后的传感器力矩 (N⋅m)
    'K_r_inv': np.array((7, 7))           # 惯性矩阵逆
}
```

#### 计算过程
```python
# 名义加速度计算
ddq_n = K_r_inv @ (tau_task - tau_s_f)   # 名义加速度 (rad/s²)

# 数值积分更新
dq_n += ddq_n * DT                       # 速度更新 (rad/s)
q_n += dq_n * DT                         # 位置更新 (rad)
```

#### 输出
```python
nominal_output = {
    'q_n': np.array(7,),                  # 更新后的名义角度 (rad)
    'dq_n': np.array(7,)                  # 更新后的名义速度 (rad/s)
}
```

### 5.3 摩擦补偿计算

#### 输入
```python
friction_input = {
    'q_n': np.array(7,),                  # 名义关节角度 (rad)
    'dq_n': np.array(7,),                 # 名义关节速度 (rad/s)
    'q_s': np.array(7,),                  # 传感器关节角度 (rad)
    'dq_s': np.array(7,)                  # 传感器关节速度 (rad/s)
}
```

#### 计算过程
```python
# 误差计算
velocity_error = dq_n - dq_s              # 速度误差 (rad/s)
position_error = q_n - q_s                # 位置误差 (rad)

# 摩擦补偿力矩
tau_f = K_r_K_l @ (velocity_error + K_lp @ position_error)
```

#### 输出
```python
tau_f = np.array(7,)                      # 摩擦补偿力矩 (N⋅m)
```

## 6. 控制参数详细说明

### 6.1 控制增益矩阵
```python
# 名义惯性矩阵 (kg⋅m²)
K_r = np.diag([0.3, 0.3, 0.3, 0.3, 0.18, 0.18, 0.18])

# 粘性摩擦系数 (N⋅m⋅s/rad)
K_l = np.diag([75.0, 75.0, 75.0, 75.0, 40.0, 40.0, 40.0])

# 位置相关摩擦系数 (N⋅m/rad)
K_lp = np.diag([5.0, 5.0, 5.0, 5.0, 4.0, 4.0, 4.0])

# 位置比例增益 (N⋅m/rad)
K_p = np.diag([100.0, 100.0, 100.0, 100.0, 50.0, 50.0, 50.0])

# 速度微分增益 (N⋅m⋅s/rad)
K_d = np.diag([3.0, 3.0, 3.0, 3.0, 2.0, 2.0, 2.0])
```

### 6.2 滤波器参数
```python
# 低通滤波器系数
ALPHA = 0.01                              # 滤波系数 [0,1]

# 滤波器方程
y[n] = α * x[n] + (1-α) * y[n-1]
```

### 6.3 时间参数
```python
# 控制周期
DT = 0.001                                # 1kHz控制频率
POLICY_CONTROL_PERIOD = 0.1               # 10Hz策略频率
```

## 7. 数据类型和单位总结

| 数据类型 | 变量名 | 形状 | 单位 | 取值范围 |
|---------|--------|------|------|----------|
| 关节角度 | q, q_d, q_n | (7,) | rad | [-π, π] |
| 关节速度 | dq, dq_d, dq_n | (7,) | rad/s | 见速度限制 |
| 关节力矩 | tau, tau_c | (7,) | N⋅m | 见力矩限制 |
| 末端位置 | arm_pos | (3,) | m | 工作空间内 |
| 末端姿态 | arm_quat | (4,) | - | 单位四元数 |
| 夹爪位置 | gripper_pos | (1,) | - | [0, 1] |
| 图像数据 | image | (84,84,3) | - | [0, 255] |

## 8. 安全机制和异常处理

### 8.1 命令流中断检测
```python
# 超时检测
timeout_threshold = 2.5 * POLICY_CONTROL_PERIOD  # 250ms
if time.time() - last_command_time > timeout_threshold:
    # 保持当前位置
    target_position = current_position
```

### 8.2 角度连续性处理
```python
# 角度展开（避免±π跳跃）
q_unwrapped = q_prev + np.mod(q_new - q_prev + np.pi, 2*np.pi) - np.pi
```

### 8.3 电流限制保护
```python
# 电流限制
current_cmd = np.clip(torque_cmd / torque_constant, 
                     current_limit_min, current_limit_max)
```

这个详细分析涵盖了整个机械臂控制系统从高层动作命令到底层硬件控制的完整数据流，包括每个环节的精确输入输出格式、数据类型、单位和取值范围。 