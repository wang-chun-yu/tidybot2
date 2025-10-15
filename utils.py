"""
工具函数模块

主要功能：
    - PID 文件管理：防止多个实例同时运行
"""

import atexit
import os
from pathlib import Path

def create_pid_file(name):
    """
    创建 PID 文件，防止多个实例同时运行
    
    功能：
        - 检查是否已有实例在运行
        - 创建 PID 文件记录当前进程 ID
        - 程序退出时自动清理 PID 文件
    
    参数:
        name: 应用名称（用于 PID 文件名）
    
    异常:
        如果已有实例在运行，抛出异常
    
    使用场景：
        - 实时控制器（如 base_controller, arm_controller）
        - 防止多个控制器同时控制同一硬件
    
    示例:
        create_pid_file('tidybot2-arm-controller')
    """
    # 检查 PID 文件是否已存在
    pid_file_path = Path(f'/tmp/{name}.pid')
    if pid_file_path.exists():
        # 从文件读取其他进程的 PID
        with open(pid_file_path, 'r', encoding='utf-8') as f:
            pid = int(f.read().strip())

        # 检查 PID 是否与当前进程匹配
        if pid != os.getpid():
            # PID 不匹配，检查其他进程是否仍在运行
            try:
                os.kill(pid, 0)  # 发送信号 0 检查进程是否存在
            except OSError:
                # 进程不存在，删除过期的 PID 文件
                print(f'Removing stale PID file (PID {pid})')
                pid_file_path.unlink()
            else:
                # 进程仍在运行，抛出异常
                raise Exception(f'Another instance of the {name} is already running (PID {pid})')

    # 将当前进程的 PID 写入文件
    pid_file_path.parent.mkdir(parents=True, exist_ok=True)
    with open(pid_file_path, 'w', encoding='utf-8') as f:
        f.write(f'{os.getpid()}\n')

    # 注册退出时的清理函数
    atexit.register(remove_pid_file, pid_file_path)

def remove_pid_file(pid_file_path):
    """
    删除 PID 文件（如果它对应当前进程）
    
    参数:
        pid_file_path: PID 文件路径
    
    注意:
        此函数通过 atexit 自动调用，一般不需要手动调用
    """
    # 仅当 PID 文件对应当前进程时才删除
    if pid_file_path.exists():
        with open(pid_file_path, 'r', encoding='utf-8') as f:
            pid = int(f.read().strip())
        if pid == os.getpid():
            pid_file_path.unlink()
