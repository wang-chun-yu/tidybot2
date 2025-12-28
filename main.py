# Author: Jimmy Wu
# Date: October 2024
#
# 主程序：协调遥操作和策略推理
# 功能：
#   - 支持仿真和真实环境
#   - 支持遥操作和策略推理模式
#   - 数据收集和保存

import argparse
import time
from itertools import count
from constants import POLICY_CONTROL_PERIOD
from episode_storage import EpisodeWriter
from policies import TeleopPolicy, RemotePolicy

def should_save_episode(writer):
    """
    询问用户是否保存当前 episode
    
    参数:
        writer: EpisodeWriter 实例，包含当前 episode 的数据
        
    返回:
        bool: True 表示保存，False 表示丢弃
    """
    if len(writer) == 0:
        print('Discarding empty episode')
        return False

    # 提示用户选择是否保存 episode
    while True:
        user_input = input('Save episode (y/n)? ').strip().lower()
        if user_input == 'y':
            return True
        if user_input == 'n':
            print('Discarding episode')
            return False
        print('Invalid response')

def run_episode(env, policy, writer=None):
    """
    运行一个完整的 episode
    
    参数:
        env: 环境实例（MujocoEnv 或 RealEnv）
        policy: 策略实例（TeleopPolicy 或 RemotePolicy）
        writer: EpisodeWriter 实例，用于保存数据（可选）
    
    流程:
        1. 重置环境
        2. 等待用户开始 episode
        3. 循环执行：获取观测 -> 查询策略 -> 执行动作
        4. Episode 结束后询问是否保存
        5. 等待用户准备重置环境
    """
    # 重置环境（机器人回到初始状态）
    print('Resetting env...')
    env.reset()
    print('Env has been reset')

    # 等待用户按下 "Start episode"
    print('Press "Start episode" in the web app when ready to start new episode')
    policy.reset()
    print('Starting new episode')

    episode_ended = False
    start_time = time.time()
    
    # 无限循环，直到用户选择重置环境
    for step_idx in count[int]():
        # 强制控制频率为 10 Hz（每 100ms 一步）
        step_end_time = start_time + step_idx * POLICY_CONTROL_PERIOD
        while time.time() < step_end_time:
            time.sleep(0.0001)  # 精确等待

        # 获取最新的观测（包括图像和状态）
        obs = env.get_obs()

        # 从策略获取动作
        action = policy.step(obs)

        # 如果遥操作未启用，则跳过
        if action is None:
            continue

        # 执行有效的动作到机器人
        if isinstance(action, dict):
            env.step(action)

            if writer is not None and not episode_ended:
                # 记录观测和执行的动作
                writer.step(obs, action)

        # Episode 结束（用户按下 "End episode"）
        elif not episode_ended and action == 'end_episode':
            episode_ended = True
            print('Episode ended')

            if writer is not None and should_save_episode(writer):
                # 在后台线程保存到磁盘（避免阻塞）
                writer.flush_async()

            print('Teleop is now active. Press "Reset env" in the web app when ready to proceed.')

        # 准备重置环境（用户按下 "Reset env"）
        elif action == 'reset_env':
            break

    if writer is not None:
        # 等待写入线程完成保存
        writer.wait_for_flush()

def main(args):
    """
    主函数：创建环境和策略，进入主循环
    
    参数:
        args: 命令行参数
            --sim: 使用仿真环境（否则使用真实机器人）
            --teleop: 使用遥操作策略（否则使用远程策略）
            --save: 保存 episode 数据
            --output-dir: 数据保存目录（默认 'data/demos'）
    
    示例:
        # 仿真 + 遥操作 + 保存数据
        python main.py --sim --teleop --save
        
        # 真实机器人 + 策略推理
        python main.py
    """
    # 创建环境
    if args.sim:
        from mujoco_env import MujocoEnv
        if args.teleop:
            env = MujocoEnv(show_images=True)  # 遥操作时显示图像
        else:
            env = MujocoEnv()  # 策略推理时不显示
    else:
        from real_env import RealEnv
        env = RealEnv()  # 真实机器人环境

    # 创建策略
    if args.teleop:
        policy = TeleopPolicy()  # 手机遥操作策略
    else:
        policy = RemotePolicy()  # 远程策略推理

    try:
        # 无限循环运行 episodes
        while True:
            # 如果需要保存数据，创建 EpisodeWriter
            print("args.output_dir: ", args.output_dir)
            writer = EpisodeWriter(args.output_dir) if args.save else None
            run_episode(env, policy, writer)
    finally:
        # 确保环境正确关闭
        env.close()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='TidyBot2 主程序 - 数据收集和策略推理')
    parser.add_argument('--sim', action='store_true', help='使用仿真环境')
    parser.add_argument('--teleop', action='store_true', help='使用遥操作模式')
    parser.add_argument('--save', action='store_true', help='保存 episode 数据')
    parser.add_argument('--output-dir', default='data/demos', help='数据保存目录')
    main(parser.parse_args())
