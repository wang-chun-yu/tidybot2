# TidyBot2 快速入门指南

欢迎接手 TidyBot2 项目！本指南将帮助您快速上手。

## 📚 文档索引

- **本文件（QUICK_START.md）**: 快速入门指南
- **[LEARNING_GUIDE.md](LEARNING_GUIDE.md)**: 详细的学习文档（强烈推荐完整阅读）
- **[README.md](README.md)**: 官方英文文档
- **[install_and_run.md](install_and_run.md)**: 安装和运行说明

## 🎯 10 分钟快速体验

### 第一步：安装环境

```bash
# 创建 conda 环境
mamba create -n tidybot2 python=3.10.14
mamba activate tidybot2

# 安装依赖
pip install -r requirements.txt
```

### 第二步：运行仿真环境

```bash
# 启动仿真环境并尝试随机动作
python -c "
import time
import numpy as np
from constants import POLICY_CONTROL_PERIOD
from mujoco_env import MujocoEnv

env = MujocoEnv(show_images=True)
env.reset()
try:
    for _ in range(100):
        action = {
            'base_pose': 0.05 * np.random.rand(3) - 0.025,
            'arm_pos': 0.1 * np.random.rand(3) + np.array([0.55, 0.0, 0.4]),
            'arm_quat': np.array([0.707, 0.707, 0.0, 0.0]),
            'gripper_pos': np.random.rand(1),
        }
        env.step(action)
        time.sleep(POLICY_CONTROL_PERIOD)
finally:
    env.close()
"
```

### 第三步：尝试手机遥操作

```bash
# 启动遥操作系统
python main.py --sim --teleop

# 1. 用手机浏览器访问终端显示的地址（例如 http://192.168.1.100:5000）
# 2. 点击 "Start AR"
# 3. 点击 "Start episode"
# 4. 使用手机控制机器人
# 5. 点击 "End episode" 结束
```

## 📖 核心概念速览

### 系统架构

```
手机 WebXR ──→ Flask ──→ TeleopPolicy ──→ 环境 ──→ 机器人
                                            ↑
                                            │
策略服务器 ──→ RemotePolicy ───────────────┘
```

### 主要模块

| 模块 | 功能 | 文件 |
|------|------|------|
| 主程序 | 数据收集和策略运行 | `main.py` |
| 仿真环境 | MuJoCo 仿真 | `mujoco_env.py` |
| 真实环境 | 真实机器人接口 | `real_env.py` |
| 底盘控制 | 全向移动底盘 | `base_controller.py` |
| 机械臂控制 | 柔顺控制 | `arm_controller.py` |
| 策略接口 | 遥操作和推理 | `policies.py` |
| 数据存储 | Episode 保存 | `episode_storage.py` |

### 关键常量

```python
# 策略控制频率
POLICY_CONTROL_FREQ = 10  # Hz（每秒 10 次）

# RPC 服务器端口
BASE_RPC_PORT = 50000  # 底盘
ARM_RPC_PORT = 50001   # 机械臂

# 策略服务器端口
POLICY_SERVER_PORT = 5555
```

## 🔧 常用命令

### 仿真模式

```bash
# 仿真 + 遥操作 + 保存数据
python main.py --sim --teleop --save

# 仿真 + 策略推理（需要先启动策略服务器）
python main.py --sim

# 重放保存的数据
python replay_episodes.py --sim --input-dir data/sim-v1

# 重放并显示图像
python replay_episodes.py --sim --input-dir data/sim-v1 --show-images
```

### 真实机器人模式

```bash
# 在 mini PC 上，打开 3 个终端

# 终端 1：启动底盘服务器
python base_server.py

# 终端 2：启动机械臂服务器
python arm_server.py

# 终端 3：运行主程序
python main.py --teleop --save  # 遥操作模式
# 或
python main.py                  # 策略推理模式
```

### 数据处理

```bash
# 转换数据格式（用于训练）
python convert_to_robomimic_hdf5.py \
    --input-dir data/my_task \
    --output-path data/my_task.hdf5
```

## 🐛 调试技巧

### 检查环境是否正常

```python
# 测试仿真环境
from mujoco_env import MujocoEnv
env = MujocoEnv()
env.reset()
obs = env.get_obs()
print(obs.keys())  # 应该包含 base_pose, arm_pos, arm_quat, gripper_pos, base_image, wrist_image
env.close()
```

### 查看相机

```python
# 测试相机
from cameras import LogitechCamera
from constants import BASE_CAMERA_SERIAL
import cv2 as cv

camera = LogitechCamera(BASE_CAMERA_SERIAL)
image = camera.get_image()
while image is None:
    image = camera.get_image()
cv.imwrite('test.jpg', cv.cvtColor(image, cv.COLOR_RGB2BGR))
print('图像已保存到 test.jpg')
```

### 底盘状态可视化

```bash
# 在 mini PC 上
python gamepad_teleop.py

# 在开发机上
ssh -L 6379:localhost:6379 minipc
python plot_base_state.py
```

## ⚠️ 常见问题

### Q: 导入错误 "ModuleNotFoundError: No module named 'kortex_api'"

A: Kinova API 仅在真实机器人上需要。在仿真模式下，不要导入 `kinova.py` 或 `arm_controller.py`。

### Q: 相机无法连接

A: 
1. 检查相机是否已连接：`ls /dev/v4l/by-id/`
2. 更新 `constants.py` 中的 `BASE_CAMERA_SERIAL`
3. 确保没有其他程序占用相机

### Q: RPC 连接失败

A: 确保对应的服务器正在运行：
```bash
# 检查进程
ps aux | grep base_server
ps aux | grep arm_server

# 检查端口
netstat -tuln | grep 50000
netstat -tuln | grep 50001
```

### Q: 底盘移动不准确

A: 
1. 检查电池电压（应 > 11.5V）
2. 校准编码器偏移（参考 `LEARNING_GUIDE.md`）
3. 检查地面摩擦力

### Q: 手机无法连接

A:
1. 确保手机和电脑在同一 WiFi
2. 检查防火墙是否阻止 5000 端口
3. 使用 HTTPS（某些浏览器需要）

## 📊 数据流图

### 遥操作数据收集

```
手机移动 → WebXR API → WebSocket → Flask 服务器 
    ↓
TeleopPolicy 处理坐标转换
    ↓
动作指令 → 环境执行 → 机器人移动
    ↓
观测 + 动作 → EpisodeWriter → 保存到磁盘
```

### 策略推理

```
相机采集图像 → 观测 → RemotePolicy → ZMQ 
    ↓
策略服务器（GPU 笔记本）
    ↓
Diffusion Policy 推理 → 动作序列 → ZMQ 
    ↓
动作指令 → 环境执行 → 机器人移动
```

## 🎓 学习路径

### 第 1 天：熟悉项目

1. ✅ 阅读 `README.md`
2. ✅ 阅读本文件（`QUICK_START.md`）
3. ✅ 运行仿真环境
4. ✅ 查看代码注释

### 第 2-3 天：数据收集

1. 练习手机遥操作
2. 设计简单任务（如拾取物体）
3. 收集 50-100 个演示
4. 验证数据质量

### 第 4-5 天：策略训练

1. 转换数据格式
2. 配置 diffusion_policy
3. 启动训练
4. 监控训练过程

### 第 6-7 天：策略部署

1. 启动策略服务器
2. 运行策略推理
3. 评估性能
4. 迭代改进

### 第 2 周：深入理解

1. 阅读 `LEARNING_GUIDE.md` 全文
2. 理解各模块实现细节
3. 尝试修改控制参数
4. 添加新功能

## 📝 代码阅读顺序

1. **入门**:
   - `constants.py` - 配置常量
   - `main.py` - 主程序流程

2. **环境**:
   - `episode_storage.py` - 数据存储
   - `mujoco_env.py` - 仿真环境
   - `real_env.py` - 真实环境

3. **策略**:
   - `policies.py` - 策略接口

4. **控制**（进阶）:
   - `base_controller.py` - 底盘控制
   - `arm_controller.py` - 机械臂控制
   - `kinova.py` - Kinova 接口

## 🔗 相关资源

- **项目主页**: http://tidybot2.github.io
- **论文**: http://tidybot2.github.io/paper.pdf
- **组装指南**: http://tidybot2.github.io/docs
- **使用指南**: http://tidybot2.github.io/docs/usage
- **视频**: https://youtu.be/6MH7dhyIUdQ

## 💡 最佳实践

### 数据收集

- ✅ 每次收集前检查机器人状态
- ✅ 从简单任务开始
- ✅ 保持演示平滑流畅
- ✅ 多样化初始状态
- ❌ 避免保存失败的演示

### 代码开发

- ✅ 修改代码前备份
- ✅ 使用 git 管理版本
- ✅ 小步修改，及时测试
- ✅ 添加注释说明
- ❌ 不要跳过测试直接部署到真实机器人

### 安全注意

- ⚠️ 测试新代码时先用仿真
- ⚠️ 真实机器人测试时降低速度
- ⚠️ 确保紧急停止按钮可用
- ⚠️ 在机器人周围保持安全距离

## 📞 获取帮助

1. **查看文档**: 先阅读 `LEARNING_GUIDE.md` 和 `README.md`
2. **搜索问题**: 在 GitHub Issues 中搜索类似问题
3. **提交 Issue**: 如果找不到答案，提交新的 Issue
4. **社区讨论**: 参与项目讨论

---

## 下一步

现在您已经完成了快速入门，建议：

1. 📖 仔细阅读 **[LEARNING_GUIDE.md](LEARNING_GUIDE.md)** 获取详细信息
2. 🎯 设定一个小目标（如完成一个简单的拾取任务）
3. 🚀 开始动手实践！

**祝您学习愉快，享受机器人研究的乐趣！** 🤖✨

