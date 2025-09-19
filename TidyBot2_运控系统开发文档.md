# TidyBot2 运控系统开发文档

## 1. 项目概述

TidyBot2是一个开源的全向移动操作机器人系统，具备强大的遥操作和模仿学习能力。该系统采用分布式架构，支持仿真和真实硬件环境。

### 1.1 系统特点
- **全向移动底盘**：采用动力脚轮实现全向运动，支持独立控制平面内所有自由度
- **7自由度机械臂**：Kinova Gen3机械臂，具备力矩控制能力
- **多模态感知**：集成底盘相机和腕部相机
- **实时控制**：250Hz底盘控制频率，1000Hz机械臂控制频率
- **分布式架构**：支持多机协同，实时控制与策略推理分离

### 1.2 硬件配置
- **移动底盘**：4个动力脚轮，TalonFX电机驱动
- **机械臂**：Kinova Gen3 7DOF，带Robotiq 2F-85夹爪
- **传感器**：Logitech C930e底盘相机，Kinova集成腕部相机
- **计算平台**：车载Mini PC + 外部GPU工作站

## 2. 系统架构

### 2.1 整体架构图
```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   手机遥控端     │    │   GPU工作站      │    │   开发机        │
│   WebXR界面     │    │   策略推理       │    │   仿真环境      │
└─────────────────┘    └─────────────────┘    └─────────────────┘
         │                        │                        │
         └────────────────────────┼────────────────────────┘
                                  │ 无线网络
         ┌────────────────────────┼────────────────────────┐
         │                        │                        │
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   车载Mini PC   │    │   底盘控制器     │    │   机械臂控制器   │
│   RPC服务器     │    │   250Hz实时     │    │   1000Hz实时    │
└─────────────────┘    └─────────────────┘    └─────────────────┘
```

### 2.2 软件模块架构
```
main.py (主程序入口)
├── 环境接口
│   ├── real_env.py (真实环境)
│   └── mujoco_env.py (仿真环境)
├── 控制系统
│   ├── base_controller.py (底盘控制器)
│   ├── arm_controller.py (机械臂控制器)
│   ├── base_server.py (底盘RPC服务器)
│   └── arm_server.py (机械臂RPC服务器)
├── 策略接口
│   ├── policies.py (策略基类和遥控策略)
│   └── policy_server.py (策略推理服务器)
├── 传感器接口
│   ├── cameras.py (相机接口)
│   └── kinova.py (机械臂底层接口)
├── 运动学求解
│   └── ik_solver.py (逆运动学求解器)
└── 数据管理
    └── episode_storage.py (数据存储)
```

## 3. 控制系统详解

### 3.1 底盘控制系统

#### 3.1.1 控制架构
```python
# 底盘控制器核心参数
CONTROL_FREQ = 250  # 250Hz控制频率
NUM_CASTERS = 4     # 4个动力脚轮
```

#### 3.1.2 运动学模型
底盘采用4个动力脚轮配置，每个脚轮包含：
- **转向电机**：控制脚轮朝向（奇数ID电机）
- **驱动电机**：控制轮子转速（偶数ID电机）

关键参数：
```python
# 脚轮几何参数
h_x, h_y = 0.190150 * [1,1,-1,-1], 0.170150 * [-1,1,1,-1]  # 脚轮位置
b_x, b_y = -0.014008, -0.003753  # 脚轮偏移
r = 0.0508  # 轮子半径
```

#### 3.1.3 控制算法
采用**在线轨迹生成（OTG）**算法，通过Ruckig库实现：
```python
# 速度和加速度限制
max_vel = (0.5, 0.5, 1.57)      # m/s, m/s, rad/s
max_accel = (0.25, 0.25, 0.79)  # m/s², m/s², rad/s²
```

控制流程：
1. **目标设定**：接收全局坐标系下的目标位姿(x, y, θ)
2. **轨迹规划**：OTG算法生成平滑轨迹
3. **运动学解算**：将目标速度转换为各脚轮的转向角和转速
4. **电机控制**：通过TalonFX电机执行控制指令

#### 3.1.4 安全机制
```python
# 电流限制（安全保护）
torque_current_limit = 40  # 转向电机：40A
torque_current_limit = 10  # 驱动电机：10A
```

### 3.2 机械臂控制系统

#### 3.2.1 控制架构
```python
# 机械臂控制器核心参数
DT = 0.001  # 1000Hz控制频率
```

#### 3.2.2 控制算法
采用**关节空间柔顺控制器（Joint Compliant Controller）**：

```python
# 控制增益矩阵
K_r = diag([0.3, 0.3, 0.3, 0.3, 0.18, 0.18, 0.18])      # 刚度矩阵
K_l = diag([75.0, 75.0, 75.0, 75.0, 40.0, 40.0, 40.0])   # 阻尼矩阵
K_p = diag([100.0, 100.0, 100.0, 100.0, 50.0, 50.0, 50.0]) # 位置增益
K_d = diag([3.0, 3.0, 3.0, 3.0, 2.0, 2.0, 2.0])         # 速度增益
```

控制方程：
```
τ = K_r⁻¹(K_l(q_d - q_n) + K_lp(q̇_d - q̇_n)) + τ_ff
```

其中：
- `q_d, q̇_d`：期望关节位置和速度
- `q_n, q̇_n`：名义关节位置和速度
- `τ_ff`：前馈力矩

#### 3.2.3 在线轨迹生成
使用Ruckig库进行关节空间轨迹规划：
```python
# 速度和加速度限制
max_velocity = [80°, 80°, 80°, 80°, 140°, 140°, 140°]      # 度/秒
max_acceleration = [240°, 240°, 240°, 240°, 450°, 450°, 450°] # 度/秒²
```

#### 3.2.4 逆运动学求解
采用阻尼最小二乘法（Damped Least Squares）：
```python
# IK求解器参数
DAMPING_COEFF = 1e-12
MAX_ANGLE_CHANGE = 45°  # 单次迭代最大角度变化
```

求解流程：
1. **误差计算**：计算末端执行器位姿误差
2. **雅可比矩阵**：计算当前配置下的雅可比矩阵
3. **更新计算**：使用阻尼最小二乘法计算关节角更新量
4. **零空间投影**：利用冗余自由度优化关节配置

## 4. 通信架构

### 4.1 RPC服务器架构
系统采用Python multiprocessing.managers实现RPC通信：

```python
# 底盘RPC服务器
class BaseManager(MPBaseManager):
    pass

BaseManager.register('Base', Base)
```

### 4.2 ZMQ策略服务器
策略推理服务器使用ZeroMQ实现：
```python
# 策略服务器配置
POLICY_SERVER_HOST = 'localhost'
POLICY_SERVER_PORT = 5555
```

### 4.3 WebXR遥控接口
手机遥控通过Flask-SocketIO实现WebXR接口：
```python
# WebXR坐标系转换
def convert_webxr_pose(pos, quat):
    # WebXR: +x右, +y上, +z后
    # 机器人: +x前, +y左, +z上
    pos = [-pos['z'], -pos['x'], pos['y']]
    rot = R.from_quat([-quat['z'], -quat['x'], quat['y'], quat['w']])
    return pos, rot
```

## 5. 传感器系统

### 5.1 相机系统

#### 5.1.1 Logitech底盘相机
```python
class LogitechCamera(Camera):
    def __init__(self, serial, frame_width=640, frame_height=360):
        # 30fps采集，MJPEG编码
        # 固定焦距，关闭自动对焦
```

#### 5.1.2 Kinova腕部相机
```python
class KinovaCamera(Camera):
    def __init__(self):
        # RTSP流，GStreamer解码
        # 低延迟配置：max-buffers=1, drop=true
```

### 5.2 机械臂状态反馈
通过Kortex API获取：
- **关节角度**：7个关节的角度反馈
- **关节速度**：7个关节的速度反馈
- **关节力矩**：7个关节的力矩反馈
- **夹爪状态**：夹爪开合程度

### 5.3 底盘状态反馈
通过Phoenix 6 API获取：
- **电机位置**：8个电机的编码器反馈
- **电机速度**：8个电机的速度反馈
- **里程计信息**：底盘位姿估计

## 6. 开发接口

### 6.1 环境接口
```python
class RealEnv:
    def __init__(self):
        # 连接RPC服务器
        self.base = base_manager.Base()
        self.arm = arm_manager.Arm()
        
    def get_obs(self):
        # 获取传感器数据
        obs = {}
        obs.update(self.base.get_state())
        obs.update(self.arm.get_state())
        obs['base_image'] = self.base_camera.get_image()
        obs['wrist_image'] = self.wrist_camera.get_image()
        return obs
        
    def step(self, action):
        # 执行动作
        self.base.execute_action(action)
        self.arm.execute_action(action)
```

### 6.2 动作空间
```python
action = {
    'base_pose': np.array([x, y, theta]),     # 底盘目标位姿
    'arm_pos': np.array([x, y, z]),           # 机械臂末端位置
    'arm_quat': np.array([x, y, z, w]),       # 机械臂末端姿态（四元数）
    'gripper_pos': np.array([pos]),           # 夹爪开合程度[0,1]
}
```

### 6.3 观测空间
```python
obs = {
    'base_pose': np.array([x, y, theta]),     # 底盘当前位姿
    'arm_pos': np.array([x, y, z]),           # 机械臂末端位置
    'arm_quat': np.array([x, y, z, w]),       # 机械臂末端姿态
    'gripper_pos': np.array([pos]),           # 夹爪开合程度
    'base_image': np.array([H, W, 3]),        # 底盘相机图像
    'wrist_image': np.array([H, W, 3]),       # 腕部相机图像
}
```

## 7. 安全机制

### 7.1 硬件安全
- **电流限制**：电机电流限制防止过载
- **紧急停止**：支持硬件急停按钮
- **碰撞检测**：建议安装碰撞传感器
- **工作区限制**：软件限制机械臂工作区域

### 7.2 软件安全
- **实时性保证**：使用实时进程优先级
- **连接监控**：RPC连接超时检测
- **状态检查**：定期检查硬件状态
- **故障恢复**：自动故障清除机制

### 7.3 操作安全
```python
# 命令流中断保护
if time.time() - self.last_command_time > 2.5 * POLICY_CONTROL_PERIOD:
    # 维持当前位置
    self.otg_inp.target_position = self.otg_out.new_position
```

## 8. 性能优化

### 8.1 实时性优化
- **进程隔离**：控制器运行在独立实时进程
- **线程限制**：限制numpy多线程避免抖动
- **内存管理**：预分配数组避免动态分配
- **通信优化**：使用共享内存和ZMQ

### 8.2 延迟隐藏
策略推理采用延迟隐藏技术：
```python
LATENCY_BUDGET = 0.2  # 200ms延迟预算
# 提前200ms启动下一次推理，确保动作序列连续
```

### 8.3 图像处理优化
- **硬件编码**：MJPEG硬件编码减少CPU负载
- **缓冲区优化**：最小缓冲区设置减少延迟
- **分辨率适配**：根据策略需求调整分辨率

## 9. 开发调试

### 9.1 仿真调试
```bash
# 启动仿真环境
python main.py --sim --teleop

# 回放数据验证
python replay_episodes.py --sim --input-dir data/sim-v1
```

### 9.2 硬件调试
```bash
# 启动底盘服务器
python base_server.py

# 启动机械臂服务器  
python arm_server.py

# 手柄调试底盘
python gamepad_teleop.py
```

### 9.3 状态可视化
```bash
# 底盘状态可视化
python plot_base_state.py

# 需要先启用Redis状态发布
# 在base_controller.py中取消注释相关代码
```

## 10. 常见问题解决

### 10.1 电机噪音问题
**现象**：电机发出嗡嗡声，运动抖动
**原因**：实时控制循环延迟抖动
**解决**：
- 检查CPU负载，确保实时进程优先级
- 避免在实时进程中导入重型库
- 使用独立进程运行控制器

### 10.2 里程计漂移
**现象**：底盘位置估计不准确
**原因**：编码器零点偏移或控制频率不稳定
**解决**：
- 校准编码器磁铁偏移量
- 确保250Hz控制频率稳定
- 检查电机供电电压

### 10.3 相机延迟
**现象**：图像延迟较大
**原因**：缓冲区设置或编解码问题
**解决**：
- 设置缓冲区大小为1
- 使用硬件编解码
- 检查USB带宽占用

### 10.4 策略推理超时
**现象**：策略服务器连接超时
**原因**：网络延迟或GPU负载过高
**解决**：
- 检查网络连接稳定性
- 监控GPU内存使用
- 调整延迟预算参数

## 11. 扩展开发

### 11.1 添加新传感器
1. 在`cameras.py`中添加新的传感器类
2. 在环境接口中集成传感器数据
3. 更新观测空间定义

### 11.2 修改控制算法
1. 继承现有控制器类
2. 重写控制回调函数
3. 调整控制参数

### 11.3 集成新机械臂
1. 实现新的机械臂接口类
2. 适配逆运动学求解器
3. 更新URDF模型文件

## 12. 参考资料

- [Kinova Gen3 API文档](https://github.com/Kinovarobotics/kortex)
- [Phoenix 6 API文档](https://v6.docs.ctr-electronics.com/)
- [Ruckig轨迹规划库](https://github.com/pantor/ruckig)
- [MuJoCo物理仿真](https://mujoco.readthedocs.io/)
- [项目主页](http://tidybot2.github.io)

---

**注意**：本文档基于TidyBot2项目代码分析编写，实际使用时请参考最新版本的代码和官方文档。在进行硬件操作时，务必遵循安全操作规程。 