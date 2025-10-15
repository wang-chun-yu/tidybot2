# Author: Jimmy Wu
# Date: October 2024
#
# Episode 数据存储和加载模块
#
# 功能：
#   - EpisodeWriter: 记录和保存 episode 数据
#   - EpisodeReader: 加载和读取 episode 数据
#
# 数据格式：
#   - 图像以 MP4 视频格式保存（节省空间）
#   - 状态和动作以 pickle 格式保存
#
# 目录结构：
#   data/demos/
#     └── 20241008T123456789/
#         ├── data.pkl           # 时间戳、观测、动作
#         ├── base_image.mp4     # 底盘相机视频
#         └── wrist_image.mp4    # 腕部相机视频

import pickle
import threading
import time
from datetime import datetime
from pathlib import Path
import cv2 as cv
import numpy as np
from constants import POLICY_CONTROL_FREQ

def write_frames_to_mp4(frames, mp4_path):
    """
    将图像帧序列写入 MP4 视频文件
    
    参数:
        frames: 图像帧列表，每帧为 RGB 格式的 numpy 数组
        mp4_path: 输出 MP4 文件路径
    """
    height, width, _ = frames[0].shape
    fourcc = cv.VideoWriter_fourcc(*'avc1')  # H.264 编码
    out = cv.VideoWriter(str(mp4_path), fourcc, POLICY_CONTROL_FREQ, (width, height))
    for frame in frames:
        bgr_frame = cv.cvtColor(frame, cv.COLOR_RGB2BGR)  # OpenCV 使用 BGR
        out.write(bgr_frame)
    out.release()

def read_frames_from_mp4(mp4_path):
    """
    从 MP4 视频文件读取图像帧序列
    
    参数:
        mp4_path: MP4 文件路径
        
    返回:
        图像帧列表，每帧为 RGB 格式的 numpy 数组
    """
    cap = cv.VideoCapture(str(mp4_path))
    frames = []
    while True:
        ret, bgr_frame = cap.read()
        if not ret:
            break
        frames.append(cv.cvtColor(bgr_frame, cv.COLOR_BGR2RGB))  # 转换为 RGB
    cap.release()
    return frames

class EpisodeWriter:
    """
    Episode 数据写入器
    
    功能：
        - 实时记录观测和动作
        - 图像保存为 MP4 视频
        - 异步写入磁盘
    
    示例：
        writer = EpisodeWriter('data/demos')
        writer.step(obs, action)
        writer.flush_async()  # 后台保存
        writer.wait_for_flush()  # 等待保存完成
    """
    def __init__(self, output_dir):
        """
        初始化 EpisodeWriter
        
        参数:
            output_dir: 输出目录路径
        """
        self.output_dir = Path(output_dir)
        # 使用时间戳创建唯一的 episode 目录
        self.episode_dir = self.output_dir / datetime.now().strftime('%Y%m%dT%H%M%S%f')
        assert not self.episode_dir.exists()

        # Episode 数据
        self.timestamps = []    # 时间戳列表
        self.observations = []  # 观测列表
        self.actions = []       # 动作列表

        # 用于异步写入的线程
        self.flush_thread = None

    def step(self, obs, action):
        """
        记录一步的观测和动作
        
        参数:
            obs: 观测字典（包含图像和状态）
            action: 动作字典
        """
        # 检查初始底盘位姿是否为零（防止底盘被推动）
        if len(self.observations) == 0 and not np.allclose(obs['base_pose'], 0.0, atol=0.01):
            raise Exception('Initial base pose should be zero. Did the base get pushed?')
        self.timestamps.append(time.time())
        self.observations.append(obs)
        self.actions.append(action)

    def __len__(self):
        """返回已记录的步数"""
        return len(self.observations)

    def _flush(self):
        """
        内部方法：将 episode 数据写入磁盘
        
        流程：
            1. 创建 episode 目录
            2. 提取图像观测
            3. 将图像写入 MP4 视频
            4. 将其余数据写入 pickle 文件
        """
        assert len(self) > 0

        # 创建 episode 目录
        self.episode_dir.mkdir(parents=True)

        # 提取图像观测（3D 数组）
        frames_dict = {}
        for obs in self.observations:
            for k, v in obs.items():
                if v.ndim == 3:  # 图像是 3D 数组 (H, W, C)
                    if k not in frames_dict:
                        frames_dict[k] = []
                    frames_dict[k].append(v)
                    obs[k] = None  # 图像将以 MP4 形式保存，这里设为 None

        # 将图像写入 MP4 视频
        for k, frames in frames_dict.items():
            mp4_path = self.episode_dir / f'{k}.mp4'
            write_frames_to_mp4(frames, mp4_path)

        # 将其余的 episode 数据写入 pickle 文件
        # 注意：pickle 不安全，只反序列化可信的数据
        with open(self.episode_dir / 'data.pkl', 'wb') as f:
            pickle.dump({'timestamps': self.timestamps, 'observations': self.observations, 'actions': self.actions}, f)
        
        # 统计已保存的 episode 数量
        num_episodes = len([child for child in self.output_dir.iterdir() if child.is_dir()])
        print(f'Saved episode to {self.episode_dir} ({num_episodes} total)')

    def flush_async(self):
        """
        异步保存 episode 数据到磁盘（在后台线程中）
        
        注意：磁盘写入可能导致低级控制器的延迟峰值
        """
        print('Saving successful episode to disk...')
        self.flush_thread = threading.Thread(target=self._flush, daemon=True)
        self.flush_thread.start()

    def wait_for_flush(self):
        """
        等待异步保存完成
        """
        if self.flush_thread is not None:
            self.flush_thread.join()
            self.flush_thread = None

class EpisodeReader:
    """
    Episode 数据读取器
    
    功能：
        - 加载 episode 数据
        - 从 MP4 视频恢复图像
        - 支持仿真和真实数据互相重放
    
    示例：
        reader = EpisodeReader(Path('data/demos/20241008T123456789'))
        for i in range(len(reader)):
            obs = reader.observations[i]
            action = reader.actions[i]
    """
    def __init__(self, episode_dir):
        """
        初始化 EpisodeReader
        
        参数:
            episode_dir: episode 目录路径
        """
        self.episode_dir = episode_dir

        # 加载数据
        # 注意：pickle 不安全，只反序列化可信的数据
        with open(episode_dir / 'data.pkl', 'rb') as f:
            data = pickle.load(f)
        self.timestamps = data['timestamps']
        self.observations = data['observations']
        self.actions = data['actions']
        assert len(self.timestamps) > 0
        assert len(self.timestamps) == len(self.observations) == len(self.actions)

        # 从 MP4 视频恢复图像观测
        frames_dict = {}
        for step_idx, obs in enumerate(self.observations):
            for k, v in obs.items():
                if v is None:  # 图像以 MP4 视频形式保存
                    # 从 MP4 文件加载图像
                    if k not in frames_dict:
                        mp4_path = episode_dir / f'{k}.mp4'
                        frames_dict[k] = read_frames_from_mp4(mp4_path)

                    # 恢复当前步的图像
                    obs[k] = frames_dict[k][step_idx]  # np.uint8

    def __len__(self):
        """返回 episode 的步数"""
        return len(self.observations)
