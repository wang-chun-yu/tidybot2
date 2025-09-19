#!/usr/bin/env python3
"""
TidyBot2 键盘遥控接口

使用键盘控制机器人，替代WebXR手机遥控
支持底盘移动、机械臂控制和夹爪操作

Author: AI Assistant  
Date: September 2024
"""

import time
import numpy as np
import threading
from queue import Queue
import sys
import termios
import tty
import select

from constants import POLICY_CONTROL_PERIOD
from policies import Policy

class KeyboardController:
    """键盘控制器类"""
    
    def __init__(self):
        self.command_queue = Queue()
        self.running = False
        self.current_mode = 'base'  # 'base' 或 'arm'
        
        # 控制参数
        self.base_step = 0.05      # 底盘移动步长 (m)
        self.base_rot_step = 0.1   # 底盘旋转步长 (rad)
        self.arm_step = 0.02       # 机械臂移动步长 (m)
        self.gripper_step = 0.1    # 夹爪步长
        
        # 当前状态
        self.base_pose = np.array([0.0, 0.0, 0.0])
        self.arm_pos = np.array([0.55, 0.0, 0.4])
        self.arm_quat = np.array([0.0, 0.0, 0.0, 1.0])
        self.gripper_pos = np.array([0.0])
        
        # 终端设置
        self.old_settings = None
    
    def start(self):
        """启动键盘监听"""
        self.running = True
        self.setup_terminal()
        
        # 启动键盘监听线程
        keyboard_thread = threading.Thread(target=self._keyboard_listener, daemon=True)
        keyboard_thread.start()
        
        print("🎮 键盘遥控已启动!")
        print("=" * 50)
        self.print_help()
    
    def stop(self):
        """停止键盘监听"""
        self.running = False
        self.restore_terminal()
        print("\n键盘遥控已停止")
    
    def setup_terminal(self):
        """设置终端为原始模式"""
        if sys.stdin.isatty():
            self.old_settings = termios.tcgetattr(sys.stdin)
            tty.setraw(sys.stdin.fileno())
    
    def restore_terminal(self):
        """恢复终端设置"""
        if self.old_settings:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
    
    def print_help(self):
        """打印帮助信息"""
        print("控制说明:")
        print("  [Tab]     - 切换控制模式 (底盘/机械臂)")
        print("  [Space]   - 开始/结束操作")
        print("  [R]       - 重置环境")
        print("  [Q]       - 退出程序")
        print()
        print("底盘控制模式:")
        print("  [W/S]     - 前进/后退")
        print("  [A/D]     - 左移/右移") 
        print("  [Q/E]     - 左转/右转")
        print()
        print("机械臂控制模式:")
        print("  [W/S]     - X轴 前/后")
        print("  [A/D]     - Y轴 左/右")
        print("  [Q/E]     - Z轴 上/下")
        print("  [Z/X]     - 夹爪 开/合")
        print()
        print(f"当前模式: {'🚗 底盘' if self.current_mode == 'base' else '🦾 机械臂'}")
        print("=" * 50)
    
    def _keyboard_listener(self):
        """键盘监听线程"""
        while self.running:
            if sys.stdin in select.select([sys.stdin], [], [], 0.01)[0]:
                key = sys.stdin.read(1).lower()
                self._process_key(key)
            time.sleep(0.01)
    
    def _process_key(self, key):
        """处理按键"""
        if key == 'q' and self.current_mode == 'base':
            # 底盘模式下Q是左转，需要特殊处理
            self._handle_base_control('q')
        elif key == '\x1b':  # ESC键退出
            self.command_queue.put('quit')
        elif key == '\t':  # Tab切换模式
            self._switch_mode()
        elif key == ' ':  # 空格开始/结束
            self.command_queue.put('toggle_episode')
        elif key == 'r':  # 重置
            self.command_queue.put('reset_env')
        elif key == '\x03':  # Ctrl+C
            self.command_queue.put('quit')
        elif self.current_mode == 'base':
            self._handle_base_control(key)
        elif self.current_mode == 'arm':
            self._handle_arm_control(key)
    
    def _switch_mode(self):
        """切换控制模式"""
        self.current_mode = 'arm' if self.current_mode == 'base' else 'base'
        mode_name = '🚗 底盘' if self.current_mode == 'base' else '🦾 机械臂'
        print(f"\r切换到: {mode_name} 控制模式", end='', flush=True)
    
    def _handle_base_control(self, key):
        """处理底盘控制"""
        if key == 'w':    # 前进
            self.base_pose[0] += self.base_step
        elif key == 's':  # 后退
            self.base_pose[0] -= self.base_step
        elif key == 'a':  # 左移
            self.base_pose[1] += self.base_step
        elif key == 'd':  # 右移
            self.base_pose[1] -= self.base_step
        elif key == 'q':  # 左转
            self.base_pose[2] += self.base_rot_step
        elif key == 'e':  # 右转
            self.base_pose[2] -= self.base_rot_step
        else:
            return
        
        # 限制范围
        self.base_pose[:2] = np.clip(self.base_pose[:2], -2.0, 2.0)
        self.base_pose[2] = np.mod(self.base_pose[2] + np.pi, 2*np.pi) - np.pi
        
        print(f"\r🚗 底盘: [{self.base_pose[0]:.2f}, {self.base_pose[1]:.2f}, {np.rad2deg(self.base_pose[2]):.1f}°]", 
              end='', flush=True)
    
    def _handle_arm_control(self, key):
        """处理机械臂控制"""
        if key == 'w':    # X轴前
            self.arm_pos[0] += self.arm_step
        elif key == 's':  # X轴后
            self.arm_pos[0] -= self.arm_step
        elif key == 'a':  # Y轴左
            self.arm_pos[1] += self.arm_step
        elif key == 'd':  # Y轴右
            self.arm_pos[1] -= self.arm_step
        elif key == 'q':  # Z轴上 (这里q不会与退出冲突)
            self.arm_pos[2] += self.arm_step
        elif key == 'e':  # Z轴下
            self.arm_pos[2] -= self.arm_step
        elif key == 'z':  # 夹爪开
            self.gripper_pos[0] = max(0.0, self.gripper_pos[0] - self.gripper_step)
        elif key == 'x':  # 夹爪合
            self.gripper_pos[0] = min(1.0, self.gripper_pos[0] + self.gripper_step)
        else:
            return
        
        # 限制工作空间
        self.arm_pos[0] = np.clip(self.arm_pos[0], 0.2, 0.8)
        self.arm_pos[1] = np.clip(self.arm_pos[1], -0.4, 0.4)
        self.arm_pos[2] = np.clip(self.arm_pos[2], 0.1, 0.8)
        
        print(f"\r🦾 机械臂: [{self.arm_pos[0]:.2f}, {self.arm_pos[1]:.2f}, {self.arm_pos[2]:.2f}] "
              f"夹爪: {self.gripper_pos[0]:.1f}", end='', flush=True)
    
    def get_current_action(self):
        """获取当前动作"""
        return {
            'base_pose': self.base_pose.copy(),
            'arm_pos': self.arm_pos.copy(),
            'arm_quat': self.arm_quat.copy(),
            'gripper_pos': self.gripper_pos.copy(),
        }
    
    def has_command(self):
        """检查是否有命令"""
        return not self.command_queue.empty()
    
    def get_command(self):
        """获取命令"""
        if not self.command_queue.empty():
            return self.command_queue.get()
        return None

class KeyboardTeleopPolicy(Policy):
    """键盘遥控策略"""
    
    def __init__(self):
        self.controller = KeyboardController()
        self.episode_active = False
        
    def reset(self):
        """重置策略"""
        print("\n等待用户按空格键开始操作...")
        self.controller.start()
        self.episode_active = False
        
        # 等待用户开始操作
        while not self.episode_active:
            if self.controller.has_command():
                cmd = self.controller.get_command()
                if cmd == 'toggle_episode':
                    self.episode_active = True
                    print("\n开始操作!")
                elif cmd == 'quit':
                    self.controller.stop()
                    exit(0)
            time.sleep(0.01)
    
    def step(self, obs):
        """执行一步策略"""
        # 检查命令
        if self.controller.has_command():
            cmd = self.controller.get_command()
            if cmd == 'toggle_episode':
                if self.episode_active:
                    print("\n结束操作!")
                    return 'end_episode'
                else:
                    self.episode_active = True
                    print("\n开始操作!")
            elif cmd == 'reset_env':
                print("\n重置环境!")
                return 'reset_env'
            elif cmd == 'quit':
                self.controller.stop()
                exit(0)
        
        # 如果操作未激活，返回None
        if not self.episode_active:
            return None
        
        # 返回当前动作
        return self.controller.get_current_action()

def main():
    """主函数 - 独立测试键盘遥控"""
    import argparse
    from mujoco_env import MujocoEnv
    
    parser = argparse.ArgumentParser(description='TidyBot2 键盘遥控测试')
    parser.add_argument('--no-images', action='store_true', help='不显示仿真图像')
    args = parser.parse_args()
    
    print("🎮 TidyBot2 键盘遥控测试")
    print("=" * 50)
    
    # 创建环境和策略
    env = MujocoEnv(show_images=not args.no_images)
    policy = KeyboardTeleopPolicy()
    
    try:
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
                    print("\n操作结束，按空格开始新操作，按R重置环境")
                    break
                elif action == 'reset_env':
                    break
                
                time.sleep(POLICY_CONTROL_PERIOD)
    
    except KeyboardInterrupt:
        print("\n程序被中断")
    finally:
        policy.controller.stop()
        env.close()
        print("程序已退出")

if __name__ == '__main__':
    main() 