#!/usr/bin/env python3
"""
TidyBot2 控制器算法测试脚本

此脚本用于测试不同的运控算法在仿真环境中的表现，
支持与手机遥控接口结合使用。

Author: AI Assistant
Date: September 2024
"""

import argparse
import time
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path

from constants import POLICY_CONTROL_PERIOD
from mujoco_env import MujocoEnv
from policies import TeleopPolicy
from ik_solver import IKSolver

class ControllerTester:
    """控制器测试类"""
    
    def __init__(self, show_images=True, record_data=True):
        self.env = MujocoEnv(show_images=show_images)
        self.policy = TeleopPolicy()
        self.ik_solver = IKSolver()
        self.record_data = record_data
        
        # 数据记录
        self.trajectory_data = {
            'time': [],
            'base_pose': [],
            'arm_pos': [],
            'arm_quat': [],
            'gripper_pos': [],
            'target_base_pose': [],
            'target_arm_pos': [],
            'target_arm_quat': [],
        }
    
    def reset_env(self):
        """重置环境"""
        print("正在重置仿真环境...")
        self.env.reset()
        self.policy.reset()
        print("环境重置完成")
        
        if self.record_data:
            # 清空数据记录
            for key in self.trajectory_data:
                self.trajectory_data[key].clear()
    
    def test_basic_control(self, duration=30):
        """测试基本控制功能"""
        print(f"\n=== 基本控制测试 (时长: {duration}秒) ===")
        print("请使用手机连接到 http://localhost:5000 进行遥控测试")
        print("测试内容:")
        print("1. 底盘位置控制")
        print("2. 机械臂位置控制") 
        print("3. 夹爪控制")
        print("4. 协调运动")
        
        self.reset_env()
        start_time = time.time()
        step_count = 0
        
        try:
            while time.time() - start_time < duration:
                # 获取观测
                obs = self.env.get_obs()
                
                # 获取遥控指令
                action = self.policy.step(obs)
                
                if action is None:
                    time.sleep(POLICY_CONTROL_PERIOD)
                    continue
                
                if isinstance(action, dict):
                    # 执行动作
                    self.env.step(action)
                    
                    # 记录数据
                    if self.record_data:
                        current_time = time.time() - start_time
                        self._record_step_data(current_time, obs, action)
                    
                    step_count += 1
                    
                    # 每秒打印一次状态
                    if step_count % 10 == 0:
                        print(f"时间: {time.time() - start_time:.1f}s, "
                              f"底盘位置: [{obs['base_pose'][0]:.3f}, {obs['base_pose'][1]:.3f}, {obs['base_pose'][2]:.3f}], "
                              f"机械臂位置: [{obs['arm_pos'][0]:.3f}, {obs['arm_pos'][1]:.3f}, {obs['arm_pos'][2]:.3f}]")
                
                elif action == 'end_episode':
                    print("用户结束了当前测试")
                    break
                elif action == 'reset_env':
                    print("用户请求重置环境")
                    self.reset_env()
                    start_time = time.time()
                    step_count = 0
                
                time.sleep(POLICY_CONTROL_PERIOD)
                
        except KeyboardInterrupt:
            print("\n用户中断测试")
        
        print(f"基本控制测试完成，共执行 {step_count} 步")
        return self.trajectory_data.copy()
    
    def test_precision_control(self, targets=None):
        """测试精确控制能力"""
        print(f"\n=== 精确控制测试 ===")
        
        if targets is None:
            # 默认测试目标点
            targets = [
                {'base_pose': [0.2, 0.0, 0.0], 'arm_pos': [0.5, 0.1, 0.4], 'description': '右前方'},
                {'base_pose': [0.0, 0.2, np.pi/4], 'arm_pos': [0.4, 0.0, 0.5], 'description': '左侧旋转'},
                {'base_pose': [-0.1, -0.1, -np.pi/4], 'arm_pos': [0.6, -0.1, 0.3], 'description': '后右方'},
                {'base_pose': [0.0, 0.0, 0.0], 'arm_pos': [0.55, 0.0, 0.4], 'description': '回到原点'},
            ]
        
        self.reset_env()
        precision_results = []
        
        for i, target in enumerate(targets):
            print(f"\n目标 {i+1}: {target['description']}")
            print(f"  底盘目标: {target['base_pose']}")
            print(f"  机械臂目标: {target['arm_pos']}")
            
            # 生成目标四元数（保持末端执行器朝下）
            target_quat = [0.0, 0.0, 0.0, 1.0]
            
            result = self._move_to_target(
                target['base_pose'], 
                target['arm_pos'], 
                target_quat,
                timeout=10.0
            )
            
            precision_results.append(result)
            print(f"  到达精度: 底盘误差={result['base_error']:.4f}m, 机械臂误差={result['arm_error']:.4f}m")
            
            time.sleep(2.0)  # 稳定时间
        
        return precision_results
    
    def test_trajectory_following(self):
        """测试轨迹跟踪能力"""
        print(f"\n=== 轨迹跟踪测试 ===")
        
        # 生成圆形轨迹
        t = np.linspace(0, 2*np.pi, 50)
        radius = 0.3
        center = [0.5, 0.0, 0.4]
        
        trajectory = []
        for i, angle in enumerate(t):
            base_pose = [0.0, 0.0, angle/4]  # 底盘缓慢旋转
            arm_pos = [
                center[0] + radius * np.cos(angle),
                center[1] + radius * np.sin(angle), 
                center[2]
            ]
            trajectory.append({
                'time': i * 0.2,
                'base_pose': base_pose,
                'arm_pos': arm_pos,
                'arm_quat': [0.0, 0.0, 0.0, 1.0]
            })
        
        print(f"执行圆形轨迹，共 {len(trajectory)} 个点")
        
        self.reset_env()
        trajectory_errors = []
        
        start_time = time.time()
        
        for i, waypoint in enumerate(trajectory):
            target_time = waypoint['time']
            
            # 等待到达目标时间
            while time.time() - start_time < target_time:
                time.sleep(0.001)
            
            # 执行轨迹点
            action = {
                'base_pose': np.array(waypoint['base_pose']),
                'arm_pos': np.array(waypoint['arm_pos']),
                'arm_quat': np.array(waypoint['arm_quat']),
                'gripper_pos': np.array([0.0])
            }
            
            self.env.step(action)
            obs = self.env.get_obs()
            
            # 计算跟踪误差
            base_error = np.linalg.norm(obs['base_pose'] - waypoint['base_pose'])
            arm_error = np.linalg.norm(obs['arm_pos'] - waypoint['arm_pos'])
            
            trajectory_errors.append({
                'time': target_time,
                'base_error': base_error,
                'arm_error': arm_error
            })
            
            if i % 10 == 0:
                print(f"  进度: {i+1}/{len(trajectory)}, 误差: 底盘={base_error:.4f}m, 机械臂={arm_error:.4f}m")
        
        print("轨迹跟踪测试完成")
        return trajectory_errors
    
    def _move_to_target(self, target_base_pose, target_arm_pos, target_arm_quat, timeout=10.0):
        """移动到目标位置"""
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            # 构造动作
            action = {
                'base_pose': np.array(target_base_pose),
                'arm_pos': np.array(target_arm_pos),
                'arm_quat': np.array(target_arm_quat),
                'gripper_pos': np.array([0.0])
            }
            
            # 执行动作
            self.env.step(action)
            obs = self.env.get_obs()
            
            # 检查是否到达目标
            base_error = np.linalg.norm(obs['base_pose'] - target_base_pose)
            arm_error = np.linalg.norm(obs['arm_pos'] - target_arm_pos)
            
            if base_error < 0.02 and arm_error < 0.02:  # 2cm精度
                return {
                    'success': True,
                    'time': time.time() - start_time,
                    'base_error': base_error,
                    'arm_error': arm_error
                }
            
            time.sleep(POLICY_CONTROL_PERIOD)
        
        # 超时
        obs = self.env.get_obs()
        return {
            'success': False,
            'time': timeout,
            'base_error': np.linalg.norm(obs['base_pose'] - target_base_pose),
            'arm_error': np.linalg.norm(obs['arm_pos'] - target_arm_pos)
        }
    
    def _record_step_data(self, current_time, obs, action):
        """记录步骤数据"""
        self.trajectory_data['time'].append(current_time)
        self.trajectory_data['base_pose'].append(obs['base_pose'].copy())
        self.trajectory_data['arm_pos'].append(obs['arm_pos'].copy())
        self.trajectory_data['arm_quat'].append(obs['arm_quat'].copy())
        self.trajectory_data['gripper_pos'].append(obs['gripper_pos'].copy())
        self.trajectory_data['target_base_pose'].append(action['base_pose'].copy())
        self.trajectory_data['target_arm_pos'].append(action['arm_pos'].copy())
        self.trajectory_data['target_arm_quat'].append(action['arm_quat'].copy())
    
    def plot_results(self, data, save_path=None):
        """绘制测试结果"""
        if not data['time']:
            print("没有数据可绘制")
            return
        
        fig, axes = plt.subplots(2, 2, figsize=(15, 10))
        fig.suptitle('TidyBot2 控制器性能测试结果', fontsize=16)
        
        time_data = np.array(data['time'])
        
        # 底盘位置
        base_poses = np.array(data['base_pose'])
        target_base_poses = np.array(data['target_base_pose'])
        
        axes[0, 0].plot(time_data, base_poses[:, 0], 'b-', label='实际 X')
        axes[0, 0].plot(time_data, target_base_poses[:, 0], 'b--', label='目标 X')
        axes[0, 0].plot(time_data, base_poses[:, 1], 'r-', label='实际 Y')  
        axes[0, 0].plot(time_data, target_base_poses[:, 1], 'r--', label='目标 Y')
        axes[0, 0].set_title('底盘位置跟踪')
        axes[0, 0].set_xlabel('时间 (s)')
        axes[0, 0].set_ylabel('位置 (m)')
        axes[0, 0].legend()
        axes[0, 0].grid(True)
        
        # 底盘角度
        axes[0, 1].plot(time_data, np.rad2deg(base_poses[:, 2]), 'g-', label='实际角度')
        axes[0, 1].plot(time_data, np.rad2deg(target_base_poses[:, 2]), 'g--', label='目标角度')
        axes[0, 1].set_title('底盘角度跟踪')
        axes[0, 1].set_xlabel('时间 (s)')
        axes[0, 1].set_ylabel('角度 (度)')
        axes[0, 1].legend()
        axes[0, 1].grid(True)
        
        # 机械臂位置
        arm_poses = np.array(data['arm_pos'])
        target_arm_poses = np.array(data['target_arm_pos'])
        
        axes[1, 0].plot(time_data, arm_poses[:, 0], 'b-', label='实际 X')
        axes[1, 0].plot(time_data, target_arm_poses[:, 0], 'b--', label='目标 X')
        axes[1, 0].plot(time_data, arm_poses[:, 1], 'r-', label='实际 Y')
        axes[1, 0].plot(time_data, target_arm_poses[:, 1], 'r--', label='目标 Y')
        axes[1, 0].plot(time_data, arm_poses[:, 2], 'g-', label='实际 Z')
        axes[1, 0].plot(time_data, target_arm_poses[:, 2], 'g--', label='目标 Z')
        axes[1, 0].set_title('机械臂位置跟踪')
        axes[1, 0].set_xlabel('时间 (s)')
        axes[1, 0].set_ylabel('位置 (m)')
        axes[1, 0].legend()
        axes[1, 0].grid(True)
        
        # 跟踪误差
        base_errors = np.linalg.norm(base_poses - target_base_poses, axis=1)
        arm_errors = np.linalg.norm(arm_poses - target_arm_poses, axis=1)
        
        axes[1, 1].plot(time_data, base_errors, 'b-', label='底盘误差')
        axes[1, 1].plot(time_data, arm_errors, 'r-', label='机械臂误差')
        axes[1, 1].set_title('位置跟踪误差')
        axes[1, 1].set_xlabel('时间 (s)')
        axes[1, 1].set_ylabel('误差 (m)')
        axes[1, 1].legend()
        axes[1, 1].grid(True)
        
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
            print(f"结果图表已保存至: {save_path}")
        else:
            plt.show()
    
    def close(self):
        """关闭环境"""
        self.env.close()

def main():
    parser = argparse.ArgumentParser(description='TidyBot2 控制器算法测试')
    parser.add_argument('--test', choices=['basic', 'precision', 'trajectory', 'all'], 
                       default='basic', help='测试类型')
    parser.add_argument('--duration', type=int, default=30, help='基本测试时长(秒)')
    parser.add_argument('--no-images', action='store_true', help='不显示仿真图像')
    parser.add_argument('--save-plot', type=str, help='保存结果图表路径')
    parser.add_argument('--no-record', action='store_true', help='不记录数据')
    
    args = parser.parse_args()
    
    print("=" * 60)
    print("TidyBot2 运控算法测试工具")
    print("=" * 60)
    print(f"测试类型: {args.test}")
    print(f"显示图像: {'否' if args.no_images else '是'}")
    print(f"记录数据: {'否' if args.no_record else '是'}")
    print()
    
    # 创建测试器
    tester = ControllerTester(
        show_images=not args.no_images,
        record_data=not args.no_record
    )
    
    try:
        if args.test == 'basic' or args.test == 'all':
            data = tester.test_basic_control(duration=args.duration)
            if not args.no_record and data['time']:
                tester.plot_results(data, args.save_plot)
        
        if args.test == 'precision' or args.test == 'all':
            precision_results = tester.test_precision_control()
            print("\n=== 精确控制测试总结 ===")
            for i, result in enumerate(precision_results):
                status = "成功" if result['success'] else "超时"
                print(f"目标 {i+1}: {status}, 用时: {result['time']:.2f}s, "
                      f"误差: 底盘={result['base_error']:.4f}m, 机械臂={result['arm_error']:.4f}m")
        
        if args.test == 'trajectory' or args.test == 'all':
            trajectory_errors = tester.test_trajectory_following()
            avg_base_error = np.mean([e['base_error'] for e in trajectory_errors])
            avg_arm_error = np.mean([e['arm_error'] for e in trajectory_errors])
            print(f"\n=== 轨迹跟踪测试总结 ===")
            print(f"平均底盘误差: {avg_base_error:.4f}m")
            print(f"平均机械臂误差: {avg_arm_error:.4f}m")
    
    except KeyboardInterrupt:
        print("\n测试被用户中断")
    except Exception as e:
        print(f"\n测试过程中出现错误: {e}")
    finally:
        tester.close()
        print("\n测试完成，环境已关闭")

if __name__ == '__main__':
    main() 