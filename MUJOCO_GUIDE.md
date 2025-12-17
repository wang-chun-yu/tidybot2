# MuJoCo 学习指南 - 面向 TidyBot2 项目

本指南将帮助您掌握 MuJoCo 物理引擎，以便更好地理解和拓展 TidyBot2 项目。

## 目录

1. [MuJoCo 简介](#mujoco-简介)
2. [核心概念](#核心概念)
3. [在 TidyBot2 中的应用](#在-tidybot2-中的应用)
4. [实践教程](#实践教程)
5. [常用 API 参考](#常用-api-参考)
6. [模型文件详解](#模型文件详解)
7. [进阶主题](#进阶主题)
8. [调试技巧](#调试技巧)
9. [学习资源](#学习资源)

---

## MuJoCo 简介

### 什么是 MuJoCo？

**MuJoCo** (Multi-Joint dynamics with Contact) 是一个快速、准确的物理引擎，专为机器人学和强化学习设计。

**核心优势：**
- ⚡ **快速**：优化的接触动力学计算
- 🎯 **准确**：精确的物理仿真
- 🔧 **灵活**：XML 建模语言
- 📊 **丰富**：完整的传感器和执行器支持
- 🐍 **易用**：Python 绑定

**在 TidyBot2 中的作用：**
- 仿真机器人运动
- 测试控制算法
- 生成训练数据
- 验证策略效果

---

## 核心概念

### 1. 模型、数据和仿真

MuJoCo 的三个核心组件：

```python
import mujoco

# 1. 模型 (MjModel) - 静态描述
model = mujoco.MjModel.from_xml_path('robot.xml')
# 包含：机器人结构、质量属性、几何形状等

# 2. 数据 (MjData) - 动态状态
data = mujoco.MjData(model)
# 包含：位置、速度、力、接触信息等

# 3. 仿真步进
mujoco.mj_step(model, data)  # 前进一个仿真步
```

### 2. 坐标系统

MuJoCo 使用右手坐标系：
- **X 轴**：向右（红色）
- **Y 轴**：向前（绿色）
- **Z 轴**：向上（蓝色）

⚠️ **注意**：这与某些机器人坐标系不同，需要做转换。

### 3. 姿态表示

MuJoCo 支持多种姿态表示：

```python
# 四元数 (w, x, y, z) - MuJoCo 内部使用
quat = data.body('end_effector').xquat  # shape: (4,)

# 旋转矩阵 (3x3)
mat = data.body('end_effector').xmat.reshape(3, 3)

# 转换
mujoco.mju_mat2Quat(quat, mat)  # 矩阵 -> 四元数
mujoco.mju_quat2Mat(mat, quat)  # 四元数 -> 矩阵
```

### 4. 主要对象类型

| 对象 | 说明 | 示例 |
|------|------|------|
| **body** | 刚体 | 机械臂连杆、底盘 |
| **joint** | 关节 | 旋转关节、滑动关节 |
| **geom** | 几何形状 | 碰撞体、视觉体 |
| **site** | 参考点 | 末端执行器位置 |
| **actuator** | 执行器 | 电机、气缸 |
| **sensor** | 传感器 | 位置、力、触觉 |
| **camera** | 相机 | 渲染视角 |

---

## 在 TidyBot2 中的应用

### 1. 模型文件结构

```
models/
├── stanford_tidybot/
│   ├── tidybot.xml          # 完整机器人模型
│   ├── base.xml             # 底盘子模型
│   └── scene.xml            # 场景配置
└── kinova_gen3/
    ├── gen3_2f85.xml        # 带夹爪的机械臂
    ├── gen3.xml             # 纯机械臂（用于 IK）
    └── scene.xml            # 测试场景
```

### 2. 仿真环境架构（mujoco_env.py）

```python
class MujocoEnv:
    def __init__(self):
        # 加载模型
        self.model = mujoco.MjModel.from_xml_path('models/stanford_tidybot/scene.xml')
        self.data = mujoco.MjData(self.model)
        
        # 多进程架构
        # - 主进程：控制逻辑
        # - 仿真进程：物理计算
        # - 渲染进程：图像生成
        
    def step(self, action):
        # 1. 更新控制指令
        # 2. 运行物理仿真
        # 3. 获取观测
```

### 3. 关键应用场景

#### 场景 1：底盘控制

```python
# models/stanford_tidybot/base.xml
<body name="base">
  <!-- 底盘位置由 3 个自由关节控制 -->
  <freejoint name="base_joint"/>  <!-- x, y, z, quat -->
  
  <!-- 简化为 x, y, theta -->
  <joint name="base_x" type="slide" axis="1 0 0"/>
  <joint name="base_y" type="slide" axis="0 1 0"/>
  <joint name="base_z" type="hinge" axis="0 0 1"/>
</body>
```

```python
# 控制底盘
class BaseController:
    def __init__(self, qpos, qvel, ctrl, timestep):
        self.qpos = qpos  # 引用 data.qpos[0:3]
        self.ctrl = ctrl  # 引用 data.ctrl[0:3]
        
    def control_callback(self, command):
        # 位置控制：直接设置目标位置
        self.ctrl[:] = command['base_pose']
```

#### 场景 2：机械臂 IK

```python
# ik_solver.py
class IKSolver:
    def __init__(self):
        # 加载纯机械臂模型（无夹爪）
        self.model = mujoco.MjModel.from_xml_path('models/kinova_gen3/gen3.xml')
        self.data = mujoco.MjData(self.model)
        
    def solve(self, target_pos, target_quat, curr_qpos):
        # 迭代求解 IK
        for _ in range(max_iters):
            # 1. 正运动学
            mujoco.mj_kinematics(self.model, self.data)
            
            # 2. 计算误差
            err = target_pos - self.data.site('end_effector').xpos
            
            # 3. 计算雅可比矩阵
            jac = np.empty((6, self.model.nv))
            mujoco.mj_jacSite(self.model, self.data, jac[:3], jac[3:], site_id)
            
            # 4. 更新关节角度
            update = jac.T @ np.linalg.solve(jac @ jac.T + damping, err)
            mujoco.mj_integratePos(self.model, self.data.qpos, update, 1.0)
```

#### 场景 3：相机渲染

```python
# mujoco_env.py - Renderer 类
class Renderer:
    def __init__(self, model, data, camera_name):
        # 设置相机
        self.camera = mujoco.MjvCamera()
        self.camera.fixedcamid = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_CAMERA, camera_name
        )
        
        # 设置渲染上下文
        self.gl_context = mujoco.gl_context.GLContext(width, height)
        self.mjr_context = mujoco.MjrContext(model, mujoco.mjtFontScale.mjFONTSCALE_150)
        
    def render(self):
        # 更新场景
        mujoco.mjv_updateScene(self.model, self.data, self.scene_option, 
                              None, self.camera, mujoco.mjtCatBit.mjCAT_ALL, self.scene)
        
        # 渲染到离屏缓冲区
        mujoco.mjr_render(self.rect, self.scene, self.mjr_context)
        
        # 读取像素
        mujoco.mjr_readPixels(self.image, None, self.rect, self.mjr_context)
```

---

## 实践教程

### 教程 1：创建简单的机器人模型

**目标**：创建一个双关节机械臂

```xml
<!-- simple_arm.xml -->
<mujoco>
  <option timestep="0.002"/>
  
  <worldbody>
    <!-- 地面 -->
    <geom type="plane" size="2 2 0.1" rgba="0.9 0.9 0.9 1"/>
    <light pos="0 0 3"/>
    
    <!-- 基座 -->
    <body name="base" pos="0 0 0.5">
      <geom type="cylinder" size="0.1 0.2" rgba="0.5 0.5 0.5 1"/>
      
      <!-- 第一个连杆 -->
      <body name="link1" pos="0 0 0.2">
        <joint name="joint1" type="hinge" axis="0 0 1" range="-180 180"/>
        <geom type="capsule" size="0.05" fromto="0 0 0 0 0 0.3" rgba="1 0 0 1"/>
        
        <!-- 第二个连杆 -->
        <body name="link2" pos="0 0 0.3">
          <joint name="joint2" type="hinge" axis="0 1 0" range="-90 90"/>
          <geom type="capsule" size="0.04" fromto="0 0 0 0 0 0.25" rgba="0 1 0 1"/>
          
          <!-- 末端 -->
          <site name="end_effector" pos="0 0 0.25" size="0.02"/>
        </body>
      </body>
    </body>
  </worldbody>
  
  <actuator>
    <motor name="motor1" joint="joint1" gear="1" ctrlrange="-5 5"/>
    <motor name="motor2" joint="joint2" gear="1" ctrlrange="-5 5"/>
  </actuator>
</mujoco>
```

**使用模型**：

```python
import mujoco
import mujoco.viewer
import numpy as np

# 加载模型
model = mujoco.MjModel.from_xml_path('simple_arm.xml')
data = mujoco.MjData(model)

# 启动可视化
with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        # 设置控制指令（正弦波）
        data.ctrl[0] = 2 * np.sin(data.time)
        data.ctrl[1] = 1 * np.cos(data.time)
        
        # 仿真步进
        mujoco.mj_step(model, data)
        
        # 同步可视化
        viewer.sync()
```

### 教程 2：读取传感器数据

```python
import mujoco
import numpy as np

model = mujoco.MjModel.from_xml_path('models/stanford_tidybot/scene.xml')
data = mujoco.MjData(model)

# 运行几步仿真
for _ in range(100):
    mujoco.mj_step(model, data)

# 1. 读取关节位置和速度
print("关节位置:", data.qpos)  # shape: (nq,)
print("关节速度:", data.qvel)  # shape: (nv,)

# 2. 读取刚体位姿
body_id = model.body('end_effector').id
body_pos = data.xpos[body_id]  # 世界坐标系位置
body_quat = data.xquat[body_id]  # 世界坐标系姿态
print(f"末端位置: {body_pos}")
print(f"末端姿态: {body_quat}")

# 3. 读取 site 位置（常用于末端执行器）
site_id = model.site('pinch_site').id
site_pos = data.site(site_id).xpos
print(f"夹持点位置: {site_pos}")

# 4. 读取接触力
for i in range(data.ncon):
    contact = data.contact[i]
    print(f"接触 {i}:")
    print(f"  力: {contact.dist}")  # 穿透深度
    print(f"  几何体 1: {contact.geom1}")
    print(f"  几何体 2: {contact.geom2}")
```

### 教程 3：计算雅可比矩阵

```python
import mujoco
import numpy as np

model = mujoco.MjModel.from_xml_path('models/kinova_gen3/gen3.xml')
data = mujoco.MjData(model)

# 设置关节角度
data.qpos[:] = np.deg2rad([0, 15, 180, -130, 0, 55, 90])

# 更新运动学
mujoco.mj_kinematics(model, data)

# 计算雅可比矩阵
site_id = model.site('pinch_site').id
jac_pos = np.zeros((3, model.nv))  # 位置雅可比
jac_rot = np.zeros((3, model.nv))  # 旋转雅可比

mujoco.mj_jacSite(model, data, jac_pos, jac_rot, site_id)

print("位置雅可比矩阵:")
print(jac_pos)
print("\n旋转雅可比矩阵:")
print(jac_rot)

# 应用：计算末端速度
joint_vel = np.random.randn(model.nv)  # 随机关节速度
end_effector_vel = jac_pos @ joint_vel  # 末端线速度
end_effector_omega = jac_rot @ joint_vel  # 末端角速度

print(f"\n末端线速度: {end_effector_vel}")
print(f"末端角速度: {end_effector_omega}")
```

### 教程 4：添加物体到场景

```python
import mujoco
import mujoco.viewer
import numpy as np

# 创建带物体的场景
xml = """
<mujoco>
  <option timestep="0.002" gravity="0 0 -9.81"/>
  
  <asset>
    <texture type="2d" name="groundplane" builtin="checker" mark="edge" 
             rgb1="0.2 0.3 0.4" rgb2="0.1 0.2 0.3" width="300" height="300"/>
    <material name="groundplane" texture="groundplane" texrepeat="5 5"/>
  </asset>
  
  <worldbody>
    <light pos="0 0 3"/>
    <geom name="floor" type="plane" size="2 2 0.1" material="groundplane"/>
    
    <!-- 可移动的立方体 -->
    <body name="box" pos="0 0 0.5">
      <freejoint/>
      <geom type="box" size="0.05 0.05 0.05" rgba="1 0 0 1" mass="0.1"/>
    </body>
    
    <!-- 固定的球体 -->
    <body name="sphere" pos="0.3 0 0.5">
      <geom type="sphere" size="0.08" rgba="0 1 0 1"/>
    </body>
  </worldbody>
</mujoco>
"""

model = mujoco.MjModel.from_xml_string(xml)
data = mujoco.MjData(model)

# 给立方体一个初始速度
data.qvel[0:3] = [0.5, 0, 1]  # vx, vy, vz

# 运行仿真
with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        mujoco.mj_step(model, data)
        viewer.sync()
```

---

## 常用 API 参考

### 仿真控制

```python
# 基本仿真步进
mujoco.mj_step(model, data)  # 完整的一步（包括所有计算）

# 分步仿真（更细粒度的控制）
mujoco.mj_step1(model, data)  # 第一阶段：位置、速度积分
mujoco.mj_step2(model, data)  # 第二阶段：约束求解、力计算

# 前向运动学
mujoco.mj_forward(model, data)  # 计算所有前向量（位置、速度等）
mujoco.mj_kinematics(model, data)  # 仅计算运动学
mujoco.mj_comPos(model, data)  # 计算质心位置

# 逆动力学
mujoco.mj_inverse(model, data)  # 计算实现当前加速度所需的力

# 重置
mujoco.mj_resetData(model, data)  # 重置到初始状态
```

### 坐标变换

```python
# 四元数操作
mujoco.mju_mulQuat(res, quat1, quat2)  # 四元数乘法
mujoco.mju_negQuat(res, quat)  # 四元数共轭（逆）
mujoco.mju_quat2Mat(mat, quat)  # 四元数 -> 旋转矩阵
mujoco.mju_mat2Quat(quat, mat)  # 旋转矩阵 -> 四元数
mujoco.mju_quat2Vel(vel, quat, dt)  # 四元数 -> 角速度

# 位置/速度积分
mujoco.mj_integratePos(model, qpos, qvel, dt)  # 位置积分
```

### 查询和索引

```python
# 通过名称获取 ID
body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'body_name')
joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'joint_name')
geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, 'geom_name')
site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, 'site_name')
camera_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, 'camera_name')

# 或者使用 Python 接口（更简洁）
body_id = model.body('body_name').id
joint_id = model.joint('joint_name').id
site_id = model.site('site_name').id

# 通过 ID 获取名称
name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body_id)
```

### 重要的 data 成员

```python
# 广义坐标
data.qpos  # 位置 (nq,)
data.qvel  # 速度 (nv,)
data.qacc  # 加速度 (nv,)

# 控制
data.ctrl  # 控制输入 (nu,)

# 刚体状态
data.xpos  # 刚体位置 (nbody, 3)
data.xquat  # 刚体姿态 (nbody, 4)
data.xmat  # 刚体旋转矩阵 (nbody, 9)

# Site 状态
data.site_xpos  # site 位置 (nsite, 3)
data.site_xmat  # site 旋转矩阵 (nsite, 9)

# 接触
data.ncon  # 接触数量
data.contact  # 接触信息数组

# 传感器
data.sensordata  # 传感器读数 (nsensordata,)
```

---

## 模型文件详解

### XML 基本结构

```xml
<mujoco model="robot_name">
  <!-- 编译选项 -->
  <compiler angle="degree" coordinate="local"/>
  
  <!-- 仿真选项 -->
  <option timestep="0.002" gravity="0 0 -9.81"/>
  
  <!-- 资源定义 -->
  <asset>
    <mesh file="mesh.stl"/>
    <texture name="tex" type="2d" file="texture.png"/>
    <material name="mat" texture="tex"/>
  </asset>
  
  <!-- 默认值 -->
  <default>
    <joint damping="1" armature="0.1"/>
    <geom contype="1" conaffinity="1"/>
  </default>
  
  <!-- 世界坐标系内容 -->
  <worldbody>
    <!-- 机器人、物体等 -->
  </worldbody>
  
  <!-- 执行器 -->
  <actuator>
    <!-- 电机、气缸等 -->
  </actuator>
  
  <!-- 传感器 -->
  <sensor>
    <!-- 位置、力、触觉等 -->
  </sensor>
</mujoco>
```

### 关节类型

```xml
<!-- 铰链关节（旋转） -->
<joint name="revolute" type="hinge" axis="0 0 1" range="-180 180"/>

<!-- 滑动关节 -->
<joint name="prismatic" type="slide" axis="1 0 0" range="0 1"/>

<!-- 球关节（3 自由度旋转） -->
<joint name="ball" type="ball"/>

<!-- 自由关节（6 自由度：3 位置 + 3 旋转） -->
<freejoint name="free"/>
```

### 执行器类型

```xml
<!-- 位置控制 -->
<position name="pos_servo" joint="joint1" kp="100"/>

<!-- 速度控制 -->
<velocity name="vel_servo" joint="joint1" kv="10"/>

<!-- 力矩控制 -->
<motor name="torque_motor" joint="joint1" gear="1" ctrlrange="-10 10"/>

<!-- 通用执行器 -->
<general joint="joint1" gainprm="100 0 0" biasprm="0 -100 0"/>
```

### TidyBot2 模型详解

#### 底盘模型 (base.xml)

```xml
<body name="base" pos="0 0 0.075">
  <!-- 底盘主体 -->
  <geom type="mesh" mesh="body" material="black"/>
  
  <!-- 4 个脚轮 -->
  <body name="caster_0" pos="0.19015 -0.17015 -0.0125">
    <!-- 转向关节 -->
    <joint name="steer_0" type="hinge" axis="0 0 1"/>
    <!-- 驱动轮 -->
    <body name="wheel_0" pos="0 0 -0.04445">
      <joint name="drive_0" type="hinge" axis="0 1 0"/>
      <geom type="cylinder" size="0.0508 0.025"/>
    </body>
  </body>
  <!-- 其他 3 个脚轮... -->
</body>
```

#### 机械臂模型 (gen3.xml)

```xml
<body name="base_link">
  <!-- 关节 1 -->
  <body name="shoulder_link">
    <joint name="joint_1" axis="0 0 1" range="-180 180"/>
    <!-- 连杆几何 -->
    <geom type="mesh" mesh="shoulder_link"/>
    
    <!-- 关节 2 -->
    <body name="half_arm_1_link">
      <joint name="joint_2" axis="0 1 0" range="-128.9 128.9"/>
      <!-- ... 更多连杆 ... -->
    </body>
  </body>
</body>
```

---

## 进阶主题

### 1. 接触动力学

```python
# 检测接触
def check_contacts(model, data, geom1_name, geom2_name):
    geom1_id = model.geom(geom1_name).id
    geom2_id = model.geom(geom2_name).id
    
    for i in range(data.ncon):
        contact = data.contact[i]
        if ((contact.geom1 == geom1_id and contact.geom2 == geom2_id) or
            (contact.geom1 == geom2_id and contact.geom2 == geom1_id)):
            return True, contact
    return False, None

# 获取接触力
def get_contact_force(model, data, contact_id):
    # 创建接触雅可比矩阵
    jac = np.zeros((6, model.nv))
    mujoco.mj_jacBodyCom(model, data, jac[:3], jac[3:], data.contact[contact_id].geom1)
    
    # 计算接触力
    force = data.contact[contact_id].frame  # 接触坐标系
    return force
```

### 2. 自定义控制器

```python
class PDController:
    """PD 控制器"""
    def __init__(self, kp, kd):
        self.kp = kp  # 位置增益
        self.kd = kd  # 速度增益
        
    def compute(self, target_pos, current_pos, current_vel):
        """计算控制力矩"""
        pos_error = target_pos - current_pos
        vel_error = 0 - current_vel  # 目标速度为 0
        torque = self.kp * pos_error + self.kd * vel_error
        return torque

# 使用
controller = PDController(kp=100, kd=10)
for _ in range(1000):
    torque = controller.compute(
        target_pos=np.deg2rad(45),
        current_pos=data.qpos[0],
        current_vel=data.qvel[0]
    )
    data.ctrl[0] = torque
    mujoco.mj_step(model, data)
```

### 3. 碰撞检测和避免

```python
# 禁用特定几何体之间的碰撞
def disable_collision(model, geom1_name, geom2_name):
    """通过设置 contype 和 conaffinity 禁用碰撞"""
    geom1_id = model.geom(geom1_name).id
    geom2_id = model.geom(geom2_name).id
    
    # 在 XML 中更好的方法：
    # <geom name="geom1" contype="1" conaffinity="2"/>
    # <geom name="geom2" contype="2" conaffinity="1"/>
    # 这样两个几何体不会碰撞

# 获取最近距离
def get_distance(model, data, geom1_name, geom2_name):
    geom1_id = model.geom(geom1_name).id
    geom2_id = model.geom(geom2_name).id
    
    # 计算距离
    dist = mujoco.mj_geomDistance(model, data, geom1_id, geom2_id)
    return dist
```

### 4. 力矩前馈控制

```python
# 重力补偿
def gravity_compensation(model, data, joint_ids):
    """计算抵消重力所需的力矩"""
    # 计算逆动力学
    mujoco.mj_inverse(model, data)
    
    # 提取重力力矩
    gravity_torque = data.qfrc_bias[joint_ids]
    return gravity_torque

# 使用
gravity_torque = gravity_compensation(model, data, range(7))
data.ctrl[:7] = desired_torque + gravity_torque
```

### 5. 多进程仿真

```python
import multiprocessing as mp
from multiprocessing import shared_memory

class ParallelSimulator:
    """多个仿真实例并行运行"""
    def __init__(self, model_path, num_workers=4):
        self.model_path = model_path
        self.num_workers = num_workers
        
    def worker(self, actions_queue, results_queue):
        model = mujoco.MjModel.from_xml_path(self.model_path)
        data = mujoco.MjData(model)
        
        while True:
            actions = actions_queue.get()
            if actions is None:
                break
                
            # 运行仿真
            data.ctrl[:] = actions
            for _ in range(100):
                mujoco.mj_step(model, data)
            
            # 返回结果
            results_queue.put(data.qpos.copy())
    
    def run_parallel(self, action_list):
        actions_queue = mp.Queue()
        results_queue = mp.Queue()
        
        # 启动工作进程
        processes = []
        for _ in range(self.num_workers):
            p = mp.Process(target=self.worker, args=(actions_queue, results_queue))
            p.start()
            processes.append(p)
        
        # 分配任务
        for actions in action_list:
            actions_queue.put(actions)
        
        # 收集结果
        results = [results_queue.get() for _ in action_list]
        
        # 终止进程
        for _ in range(self.num_workers):
            actions_queue.put(None)
        for p in processes:
            p.join()
        
        return results
```

---

## 调试技巧

### 1. 可视化调试

```python
import mujoco.viewer

# 启动交互式查看器
model = mujoco.MjModel.from_xml_path('robot.xml')
data = mujoco.MjData(model)

with mujoco.viewer.launch_passive(model, data) as viewer:
    # 设置可视化选项
    viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = True  # 显示接触点
    viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTFORCE] = True  # 显示接触力
    viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_TRANSPARENT] = False  # 禁用透明
    
    while viewer.is_running():
        mujoco.mj_step(model, data)
        viewer.sync()
```

### 2. 添加调试几何体

```python
def add_debug_sphere(viewer, pos, size=0.05, rgba=[1, 0, 0, 1]):
    """在场景中添加调试球体"""
    # 在运行时添加几何体比较复杂
    # 更好的方法是在 XML 中预定义，然后控制可见性
    pass

# 在 XML 中预定义调试几何体
"""
<body name="debug_marker" pos="0 0 0" mocap="true">
  <geom type="sphere" size="0.05" rgba="1 0 0 0.5" contype="0" conaffinity="0"/>
</body>
"""

# 在代码中移动调试标记
debug_body_id = model.body('debug_marker').id
data.mocap_pos[debug_body_id] = target_position
```

### 3. 记录和回放

```python
class Recorder:
    """记录仿真轨迹"""
    def __init__(self):
        self.trajectory = []
    
    def record(self, data):
        self.trajectory.append({
            'qpos': data.qpos.copy(),
            'qvel': data.qvel.copy(),
            'ctrl': data.ctrl.copy(),
            'time': data.time,
        })
    
    def replay(self, model, viewer):
        data = mujoco.MjData(model)
        for state in self.trajectory:
            data.qpos[:] = state['qpos']
            data.qvel[:] = state['qvel']
            mujoco.mj_forward(model, data)
            viewer.sync()
            time.sleep(model.opt.timestep)

# 使用
recorder = Recorder()
for _ in range(1000):
    mujoco.mj_step(model, data)
    recorder.record(data)

# 回放
with mujoco.viewer.launch_passive(model, data) as viewer:
    recorder.replay(model, viewer)
```

### 4. 性能分析

```python
import time

def benchmark_simulation(model, num_steps=10000):
    """测试仿真性能"""
    data = mujoco.MjData(model)
    
    start = time.time()
    for _ in range(num_steps):
        mujoco.mj_step(model, data)
    elapsed = time.time() - start
    
    print(f"仿真 {num_steps} 步耗时: {elapsed:.3f} 秒")
    print(f"实时因子: {(num_steps * model.opt.timestep) / elapsed:.2f}x")
    print(f"每步耗时: {elapsed / num_steps * 1000:.3f} ms")

benchmark_simulation(model)
```

### 5. 检查模型有效性

```python
def check_model(model):
    """检查模型配置"""
    print(f"自由度 (nv): {model.nv}")
    print(f"广义坐标 (nq): {model.nq}")
    print(f"执行器 (nu): {model.nu}")
    print(f"刚体 (nbody): {model.nbody}")
    print(f"关节 (njnt): {model.njnt}")
    print(f"几何体 (ngeom): {model.ngeom}")
    print(f"时间步长: {model.opt.timestep}")
    print(f"重力: {model.opt.gravity}")
    
    # 检查关节限位
    for i in range(model.njnt):
        jnt_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
        jnt_range = model.jnt_range[i]
        print(f"关节 {jnt_name}: 范围 {jnt_range}")

check_model(model)
```

---

## 学习资源

### 官方资源

1. **官方文档**：https://mujoco.readthedocs.io/
   - API 参考
   - 建模指南
   - 编程指南

2. **官方教程**：https://github.com/deepmind/mujoco
   - 示例代码
   - 基础教程

3. **论坛**：https://github.com/deepmind/mujoco/discussions
   - 社区问答
   - 最佳实践

### 学习路径

#### 初级（1-2 周）
- ✅ 安装和基本使用
- ✅ 理解模型、数据、仿真
- ✅ 创建简单的机器人模型
- ✅ 运行仿真和可视化
- ✅ 读取传感器数据

#### 中级（2-4 周）
- ✅ 编写自定义控制器
- ✅ 理解接触动力学
- ✅ 计算雅可比矩阵和 IK
- ✅ 相机渲染
- ✅ 多进程架构

#### 高级（1-2 个月）
- ✅ 优化仿真性能
- ✅ 复杂的接触场景
- ✅ 自定义传感器
- ✅ 与深度学习框架集成
- ✅ 分布式仿真

### 推荐项目

1. **robosuite**：https://github.com/ARISE-Initiative/robosuite
   - 机器人操作环境
   - 大量示例任务

2. **dm_control**：https://github.com/deepmind/dm_control
   - DeepMind 的控制套件
   - 强化学习环境

3. **MuJoCo Menagerie**：https://github.com/deepmind/mujoco_menagerie
   - 高质量机器人模型库
   - 包含各种真实机器人

### 相关论文

1. **MuJoCo 原始论文**：
   - Todorov, E., Erez, T., & Tassa, Y. (2012). MuJoCo: A physics engine for model-based control.

2. **接触动力学**：
   - Todorov, E. (2014). Convex and analytically-invertible dynamics with contacts and constraints.

### 书籍推荐

1. **《Robotics: Modelling, Planning and Control》** by Bruno Siciliano
   - 机器人学基础
   - 运动学和动力学

2. **《Modern Robotics》** by Kevin Lynch
   - 现代机器人学
   - 在线免费课程

---

## 实践项目建议

### 项目 1：改进 TidyBot2 仿真

**目标**：为 TidyBot2 添加更真实的物理效果

**任务**：
1. 添加地面摩擦力模型
2. 模拟脚轮的打滑
3. 添加传感器噪声
4. 改进碰撞检测

### 项目 2：新任务场景

**目标**：创建新的操作任务

**任务**：
1. 设计任务场景（如桌面整理）
2. 添加物体（杯子、书本等）
3. 定义任务成功标准
4. 收集演示数据

### 项目 3：可视化工具

**目标**：开发调试和分析工具

**任务**：
1. 实时轨迹可视化
2. 力/力矩显示
3. 性能监控面板
4. 数据记录和回放

### 项目 4：物理验证

**目标**：验证仿真与真实机器人的一致性

**任务**：
1. 同时记录仿真和真实轨迹
2. 比较位置、速度误差
3. 分析差异原因
4. 调整仿真参数

---

## 常见问题（FAQ）

### Q1: 仿真速度太慢怎么办？

**A**:
1. 减少接触点数量
2. 简化几何体（用简单形状代替 mesh）
3. 增大时间步长（注意稳定性）
4. 禁用不必要的可视化
5. 使用多进程并行仿真

### Q2: 仿真不稳定怎么办？

**A**:
1. 减小时间步长
2. 增加阻尼（damping）
3. 调整接触参数（solimp, solref）
4. 检查模型配置（质量、惯性）
5. 使用更稳定的求解器

### Q3: 如何调试 IK 不收敛？

**A**:
1. 可视化目标位姿和当前位姿
2. 检查目标是否在工作空间内
3. 增加迭代次数
4. 调整阻尼系数
5. 使用更好的初始猜测

### Q4: 接触力不真实怎么办？

**A**:
1. 调整 solimp（接触刚度）
2. 调整 solref（接触阻尼）
3. 增加摩擦系数
4. 减小穿透深度
5. 使用更小的时间步长

### Q5: 如何添加自己的机器人？

**A**:
1. 从 URDF 转换：使用 `mujoco.MjModel.from_xml_string(urdf)`
2. 手动编写 XML
3. 调整关节限位、质量属性
4. 添加碰撞几何体
5. 配置执行器

---

## 总结

MuJoCo 是一个强大而灵活的物理引擎，掌握它需要：

1. **理论基础**：运动学、动力学、接触力学
2. **实践经验**：多写代码、多调试
3. **持续学习**：关注官方更新、阅读论文

**学习建议**：
- 🎯 从简单示例开始
- 📖 仔细阅读官方文档
- 💻 多动手实践
- 🤝 参与社区讨论
- 🔍 深入理解 TidyBot2 代码

**祝您学习顺利！** 🚀

如有问题，欢迎查阅：
- 📚 [LEARNING_GUIDE.md](LEARNING_GUIDE.md)
- 🚀 [QUICK_START.md](QUICK_START.md)
- 📖 [README.md](README.md)

