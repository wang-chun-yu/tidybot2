"""
全局常量配置文件

包含以下配置：
1. 移动底盘参数
2. RPC 服务器配置
3. 相机配置
4. 策略配置
"""

import numpy as np

################################################################################
# 移动底盘配置

# 车体中心到转向轴的距离 (单位: 米)
# 用于底盘运动学计算
# 4 个脚轮的布局：
#   脚轮 0: 右前 (+h_x, -h_y)
#   脚轮 1: 左前 (+h_x, +h_y)
#   脚轮 2: 左后 (-h_x, +h_y)
#   脚轮 3: 右后 (-h_x, -h_y)
h_x, h_y = 0.190150 * np.array([1.0, 1.0, -1.0, -1.0]), 0.170150 * np.array([-1.0, 1.0, 1.0, -1.0])  # Kinova / Franka
# h_x, h_y = 0.140150 * np.array([1.0, 1.0, -1.0, -1.0]), 0.120150 * np.array([-1.0, 1.0, 1.0, -1.0])  # ARX5

# 编码器磁铁偏移量
# TODO: 需要校准，参考 LEARNING_GUIDE.md 中的校准方法
ENCODER_MAGNET_OFFSETS = [0.0 / 4096, 0.0 / 4096, 0.0 / 4096, 0.0 / 4096]

################################################################################
# 遥操作和模仿学习配置

# 底盘和机械臂 RPC 服务器配置
# 用于隔离实时控制器和非实时代码
BASE_RPC_HOST = 'localhost'  # 底盘服务器主机
BASE_RPC_PORT = 50000        # 底盘服务器端口
ARM_RPC_HOST = 'localhost'   # 机械臂服务器主机
ARM_RPC_PORT = 50001         # 机械臂服务器端口
RPC_AUTHKEY = b'secret password'  # RPC 认证密钥

# 相机配置
# BASE_CAMERA_SERIAL: Logitech C930e 相机的序列号
# 可以通过 ls /dev/v4l/by-id/ 查找
BASE_CAMERA_SERIAL = 'TODO'  # TODO: 替换为实际的相机序列号
# WRIST_CAMERA_SERIAL = 'TODO'  # Kinova 腕部相机不需要序列号

# 策略配置
POLICY_SERVER_HOST = 'localhost'  # 策略服务器主机（通常在 GPU 笔记本上）
POLICY_SERVER_PORT = 5555         # 策略服务器端口
POLICY_CONTROL_FREQ = 10          # 策略控制频率 (Hz)
POLICY_CONTROL_PERIOD = 1.0 / POLICY_CONTROL_FREQ  # 控制周期 (秒)
POLICY_IMAGE_WIDTH = 84           # 策略输入图像宽度（会从原始图像缩放）
POLICY_IMAGE_HEIGHT = 84          # 策略输入图像高度（会从原始图像缩放）
