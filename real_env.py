# Author: Jimmy Wu
# Date: October 2024
#
# 真实机器人环境接口
#
# 功能：
#   - 连接底盘和机械臂 RPC 服务器
#   - 封装相机接口
#   - 提供与仿真环境一致的 API
#
# 使用前提：
#   - 必须先启动 base_server.py
#   - 必须先启动 arm_server.py
#
# 示例：
#   env = RealEnv()
#   env.reset()
#   obs = env.get_obs()
#   env.step(action)
#   env.close()

from cameras import KinovaCamera, LogitechCamera
from constants import BASE_RPC_HOST, BASE_RPC_PORT, ARM_RPC_HOST, ARM_RPC_PORT, RPC_AUTHKEY
from constants import BASE_CAMERA_SERIAL
from arm_server import ArmManager
from base_server import BaseManager

class RealEnv:
    def __init__(self):
        """
        初始化真实机器人环境
        
        连接：
            - 底盘 RPC 服务器（端口 50000）
            - 机械臂 RPC 服务器（端口 50001）
            - 底盘相机（Logitech C930e）
            - 腕部相机（Kinova 内置）
        
        异常：
            如果 RPC 服务器未运行，抛出异常
        """
        # 连接底盘 RPC 服务器
        base_manager = BaseManager(address=(BASE_RPC_HOST, BASE_RPC_PORT), authkey=RPC_AUTHKEY)
        try:
            base_manager.connect()
        except ConnectionRefusedError as e:
            raise Exception('Could not connect to base RPC server, is base_server.py running?') from e

        # 连接机械臂 RPC 服务器
        arm_manager = ArmManager(address=(ARM_RPC_HOST, ARM_RPC_PORT), authkey=RPC_AUTHKEY)
        try:
            arm_manager.connect()
        except ConnectionRefusedError as e:
            raise Exception('Could not connect to arm RPC server, is arm_server.py running?') from e

        # 创建 RPC 代理对象
        # max_vel: 最大速度 (vx, vy, omega)，单位: m/s, m/s, rad/s
        # max_accel: 最大加速度，单位: m/s², m/s², rad/s²
        self.base = base_manager.Base(max_vel=(0.5, 0.5, 1.57), max_accel=(0.5, 0.5, 1.57))
        self.arm = arm_manager.Arm()

        # 连接相机
        self.base_camera = LogitechCamera(BASE_CAMERA_SERIAL)
        self.wrist_camera = KinovaCamera()

    def get_obs(self):
        """
        获取当前观测
        
        返回:
            obs: 观测字典，包含：
                - base_pose: 底盘位姿 (x, y, theta)
                - arm_pos: 机械臂末端位置 (x, y, z)
                - arm_quat: 机械臂末端姿态 (x, y, z, w)
                - gripper_pos: 夹爪位置 [0, 1]
                - base_image: 底盘相机图像 (640, 360, 3)
                - wrist_image: 腕部相机图像 (640, 480, 3)
        """
        obs = {}
        obs.update(self.base.get_state())  # 底盘状态
        obs.update(self.arm.get_state())   # 机械臂状态
        obs['base_image'] = self.base_camera.get_image()    # 底盘相机
        obs['wrist_image'] = self.wrist_camera.get_image()  # 腕部相机
        return obs

    def reset(self):
        """
        重置机器人到初始状态
        
        流程：
            1. 重置底盘位姿到原点 (0, 0, 0)
            2. 重置机械臂到 retract 姿态
        """
        print('Resetting base...')
        self.base.reset()

        print('Resetting arm...')
        self.arm.reset()

        print('Robot has been reset')

    def step(self, action):
        """
        执行动作
        
        参数:
            action: 动作字典，包含：
                - base_pose: 底盘目标位姿 (相对当前位置的增量)
                - arm_pos: 机械臂末端目标位置 (绝对坐标)
                - arm_quat: 机械臂末端目标姿态 (四元数)
                - gripper_pos: 夹爪目标位置 [0, 1]
        
        注意：
            不返回观测，以避免策略使用过时的数据
            底盘和机械臂动作非阻塞执行
        """
        # 注意：我们故意不在这里返回 obs，以防止策略使用过时的数据
        self.base.execute_action(action)  # 非阻塞
        self.arm.execute_action(action)   # 非阻塞

    def close(self):
        """
        关闭环境，释放资源
        """
        self.base.close()
        self.arm.close()
        self.base_camera.close()
        self.wrist_camera.close()

if __name__ == '__main__':
    import time
    import numpy as np
    from constants import POLICY_CONTROL_PERIOD
    env = RealEnv()
    try:
        while True:
            env.reset()
            for _ in range(100):
                action = {
                    'base_pose': 0.1 * np.random.rand(3) - 0.05,
                    'arm_pos': 0.1 * np.random.rand(3) + np.array([0.55, 0.0, 0.4]),
                    'arm_quat': np.random.rand(4),
                    'gripper_pos': np.random.rand(1),
                }
                env.step(action)
                obs = env.get_obs()
                print([(k, v.shape) if v.ndim == 3 else (k, v) for (k, v) in obs.items()])
                time.sleep(POLICY_CONTROL_PERIOD)  # Note: Not precise
    finally:
        env.close()
