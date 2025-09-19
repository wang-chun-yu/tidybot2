#!/usr/bin/env python3
"""
TidyBot2 增强版游戏手柄遥控接口

使用游戏手柄（如Xbox、PS4控制器）控制机器人
提供更直观的控制体验，替代WebXR手机遥控

Author: AI Assistant
Date: September 2024
"""

import time
import numpy as np
import pygame
from pygame.joystick import Joystick

from constants import POLICY_CONTROL_PERIOD
from policies import Policy

class GamepadController:
    """游戏手柄控制器类"""
    
    def __init__(self, deadzone=0.15):
        pygame.init()
        pygame.joystick.init()
        
        # 检查手柄连接
        if pygame.joystick.get_count() == 0:
            raise RuntimeError("未检测到游戏手柄！请连接手柄后重试。")
        
        self.joystick = Joystick(0)
        self.joystick.init()
        self.deadzone = deadzone
        
        print(f"🎮 检测到游戏手柄: {self.joystick.get_name()}")
        print(f"   轴数量: {self.joystick.get_numaxes()}")
        print(f"   按钮数量: {self.joystick.get_numbuttons()}")
        
        # 控制参数
        self.base_max_vel = 0.3        # 底盘最大速度 (m/s)
        self.base_max_rot = 1.0        # 底盘最大角速度 (rad/s)
        self.arm_max_vel = 0.1         # 机械臂最大速度 (m/s)
        self.gripper_speed = 0.5       # 夹爪速度
        
        # 当前状态
        self.base_pose = np.array([0.0, 0.0, 0.0])
        self.arm_pos = np.array([0.55, 0.0, 0.4])
        self.arm_quat = np.array([0.0, 0.0, 0.0, 1.0])
        self.gripper_pos = np.array([0.0])
        
        # 控制模式
        self.control_mode = 'base'  # 'base' 或 'arm'
        self.last_mode_switch = 0
        
        # 按钮状态
        self.button_states = {}
        self.last_button_states = {}
    
    def apply_deadzone(self, value):
        """应用死区"""
        if abs(value) < self.deadzone:
            return 0.0
        sign = 1 if value > 0 else -1
        return sign * (abs(value) - self.deadzone) / (1.0 - self.deadzone)
    
    def update(self, dt):
        """更新控制器状态"""
        pygame.event.pump()
        
        # 更新按钮状态
        self.last_button_states = self.button_states.copy()
        self.button_states = {
            i: self.joystick.get_button(i) 
            for i in range(self.joystick.get_numbuttons())
        }
        
        # 检查模式切换 (Y按钮或三角按钮)
        if self._button_pressed(3):  # Y/Triangle按钮
            current_time = time.time()
            if current_time - self.last_mode_switch > 0.5:  # 防抖
                self.control_mode = 'arm' if self.control_mode == 'base' else 'base'
                self.last_mode_switch = current_time
                mode_name = '🚗 底盘' if self.control_mode == 'base' else '🦾 机械臂'
                print(f"\n切换到: {mode_name} 控制模式")
        
        # 根据当前模式更新状态
        if self.control_mode == 'base':
            self._update_base_control(dt)
        else:
            self._update_arm_control(dt)
    
    def _button_pressed(self, button_id):
        """检查按钮是否刚被按下"""
        return (self.button_states.get(button_id, False) and 
                not self.last_button_states.get(button_id, False))
    
    def _update_base_control(self, dt):
        """更新底盘控制"""
        # 左摇杆控制X/Y移动
        left_x = self.apply_deadzone(self.joystick.get_axis(0))  # 左右
        left_y = self.apply_deadzone(self.joystick.get_axis(1))  # 前后
        
        # 右摇杆控制旋转
        right_x = self.apply_deadzone(self.joystick.get_axis(2))  # 旋转
        
        # 更新底盘位置
        self.base_pose[0] += -left_y * self.base_max_vel * dt  # Y轴反向
        self.base_pose[1] += left_x * self.base_max_vel * dt
        self.base_pose[2] += right_x * self.base_max_rot * dt
        
        # 限制范围
        self.base_pose[:2] = np.clip(self.base_pose[:2], -2.0, 2.0)
        self.base_pose[2] = np.mod(self.base_pose[2] + np.pi, 2*np.pi) - np.pi
        
        # 显示状态
        if abs(left_x) > 0.1 or abs(left_y) > 0.1 or abs(right_x) > 0.1:
            print(f"\r🚗 底盘: [{self.base_pose[0]:.2f}, {self.base_pose[1]:.2f}, "
                  f"{np.rad2deg(self.base_pose[2]):.1f}°]", end='', flush=True)
    
    def _update_arm_control(self, dt):
        """更新机械臂控制"""
        # 左摇杆控制X/Y移动
        left_x = self.apply_deadzone(self.joystick.get_axis(0))
        left_y = self.apply_deadzone(self.joystick.get_axis(1))
        
        # 右摇杆Y轴控制Z移动
        right_y = self.apply_deadzone(self.joystick.get_axis(3))
        
        # 肩部按钮控制夹爪
        left_trigger = self.joystick.get_axis(4) if self.joystick.get_numaxes() > 4 else 0  # LT
        right_trigger = self.joystick.get_axis(5) if self.joystick.get_numaxes() > 5 else 0  # RT
        
        # 更新机械臂位置
        self.arm_pos[0] += -left_y * self.arm_max_vel * dt
        self.arm_pos[1] += left_x * self.arm_max_vel * dt
        self.arm_pos[2] += -right_y * self.arm_max_vel * dt
        
        # 更新夹爪（扳机键控制）
        trigger_diff = (right_trigger + 1) / 2 - (left_trigger + 1) / 2  # 转换到0-1范围
        self.gripper_pos[0] += trigger_diff * self.gripper_speed * dt
        self.gripper_pos[0] = np.clip(self.gripper_pos[0], 0.0, 1.0)
        
        # 限制工作空间
        self.arm_pos[0] = np.clip(self.arm_pos[0], 0.2, 0.8)
        self.arm_pos[1] = np.clip(self.arm_pos[1], -0.4, 0.4)
        self.arm_pos[2] = np.clip(self.arm_pos[2], 0.1, 0.8)
        
        # 显示状态
        if (abs(left_x) > 0.1 or abs(left_y) > 0.1 or abs(right_y) > 0.1 or 
            abs(trigger_diff) > 0.1):
            print(f"\r🦾 机械臂: [{self.arm_pos[0]:.2f}, {self.arm_pos[1]:.2f}, "
                  f"{self.arm_pos[2]:.2f}] 夹爪: {self.gripper_pos[0]:.2f}", 
                  end='', flush=True)
    
    def get_current_action(self):
        """获取当前动作"""
        return {
            'base_pose': self.base_pose.copy(),
            'arm_pos': self.arm_pos.copy(),
            'arm_quat': self.arm_quat.copy(),
            'gripper_pos': self.gripper_pos.copy(),
        }
    
    def is_start_pressed(self):
        """检查开始按钮是否被按下"""
        return self._button_pressed(7)  # Start按钮
    
    def is_select_pressed(self):
        """检查选择按钮是否被按下"""
        return self._button_pressed(6)  # Select/Back按钮
    
    def is_a_pressed(self):
        """检查A按钮是否被按下"""
        return self._button_pressed(0)  # A/X按钮
    
    def print_help(self):
        """打印帮助信息"""
        print("\n🎮 游戏手柄控制说明:")
        print("=" * 50)
        print("通用控制:")
        print("  Start按钮    - 开始/结束操作")
        print("  Select按钮   - 重置环境")
        print("  Y/△按钮     - 切换控制模式")
        print("  A/×按钮     - 退出程序")
        print()
        print("底盘控制模式 (🚗):")
        print("  左摇杆      - 前后左右移动")
        print("  右摇杆X轴   - 旋转")
        print()
        print("机械臂控制模式 (🦾):")
        print("  左摇杆      - X/Y轴移动")
        print("  右摇杆Y轴   - Z轴移动")
        print("  LT/RT扳机   - 夹爪开合")
        print()
        print(f"当前模式: {'🚗 底盘' if self.control_mode == 'base' else '🦾 机械臂'}")
        print("=" * 50)

class GamepadTeleopPolicy(Policy):
    """游戏手柄遥控策略"""
    
    def __init__(self):
        try:
            self.controller = GamepadController()
            self.episode_active = False
            self.last_update_time = time.time()
        except RuntimeError as e:
            print(f"❌ 错误: {e}")
            print("\n请检查以下事项:")
            print("1. 游戏手柄是否正确连接")
            print("2. 手柄驱动是否安装")
            print("3. 尝试重新插拔手柄")
            raise
    
    def reset(self):
        """重置策略"""
        print("\n等待用户按Start按钮开始操作...")
        self.controller.print_help()
        self.episode_active = False
        self.last_update_time = time.time()
        
        # 等待用户开始操作
        while not self.episode_active:
            current_time = time.time()
            dt = current_time - self.last_update_time
            self.last_update_time = current_time
            
            self.controller.update(dt)
            
            if self.controller.is_start_pressed():
                self.episode_active = True
                print("\n🚀 开始操作!")
            elif self.controller.is_a_pressed():
                print("\n程序退出")
                exit(0)
            
            time.sleep(0.01)
    
    def step(self, obs):
        """执行一步策略"""
        current_time = time.time()
        dt = current_time - self.last_update_time
        self.last_update_time = current_time
        
        # 更新控制器状态
        self.controller.update(dt)
        
        # 检查控制指令
        if self.controller.is_start_pressed():
            if self.episode_active:
                print("\n⏹️ 结束操作!")
                return 'end_episode'
            else:
                self.episode_active = True
                print("\n🚀 开始操作!")
        elif self.controller.is_select_pressed():
            print("\n🔄 重置环境!")
            return 'reset_env'
        elif self.controller.is_a_pressed():
            print("\n👋 程序退出")
            exit(0)
        
        # 如果操作未激活，返回None
        if not self.episode_active:
            return None
        
        # 返回当前动作
        return self.controller.get_current_action()

def main():
    """主函数 - 独立测试游戏手柄遥控"""
    import argparse
    from mujoco_env import MujocoEnv
    
    parser = argparse.ArgumentParser(description='TidyBot2 游戏手柄遥控测试')
    parser.add_argument('--no-images', action='store_true', help='不显示仿真图像')
    args = parser.parse_args()
    
    print("🎮 TidyBot2 游戏手柄遥控测试")
    print("=" * 50)
    
    try:
        # 创建环境和策略
        env = MujocoEnv(show_images=not args.no_images)
        policy = GamepadTeleopPolicy()
        
        while True:
            # 重置环境
            env.reset()
            policy.reset()
            
            # 运行操作循环
            while True:
                obs = env.get_obs()
                action = policy.step(obs)
                
                if action is None:
                    time.sleep(POLICY_CONTROL_PERIOD)
                    continue
                elif isinstance(action, dict):
                    env.step(action)
                elif action == 'end_episode':
                    print("\n操作结束，按Start开始新操作，按Select重置环境")
                    break
                elif action == 'reset_env':
                    break
                
                time.sleep(POLICY_CONTROL_PERIOD)
    
    except KeyboardInterrupt:
        print("\n程序被中断")
    except Exception as e:
        print(f"\n❌ 发生错误: {e}")
    finally:
        try:
            env.close()
        except:
            pass
        pygame.quit()
        print("程序已退出")

if __name__ == '__main__':
    main() 