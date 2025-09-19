# Author: Jimmy Wu
# Date: October 2024
#
# 机械臂关节空间顺应性控制器
# 
# 控制原理说明：
# 本控制器实现了基于阻抗控制的关节空间顺应性控制算法。
# 核心思想是通过虚拟的名义（nominal）动力学模型来实现柔顺控制，
# 使机械臂能够在执行任务时对外力产生合适的响应。
#
# 算法架构：
# 1. 任务空间控制：计算完成指定任务所需的关节力矩
# 2. 名义动力学：建立虚拟的机械臂动力学模型
# 3. 摩擦补偿：补偿实际系统与名义模型之间的差异
# 4. 在线轨迹生成：平滑地生成从当前位置到目标位置的轨迹
#
# References:
# - https://github.com/empriselab/gen3_compliant_controllers/blob/main/src/JointSpaceCompliantController.cpp
# - https://github.com/empriselab/gen3_compliant_controllers/blob/main/media/controller_formulation.pdf

import math
import time
import numpy as np
from ruckig import InputParameter, OutputParameter, Result, Ruckig
from constants import POLICY_CONTROL_PERIOD
from kinova import TorqueControlledArm

# ==================== 控制器参数配置 ====================

# 低通滤波器系数，用于平滑力矩传感器读数
ALPHA = 0.01

# 名义动力学参数矩阵 - 定义虚拟机械臂的惯性特性
# 对角矩阵，每个关节对应一个惯性参数
# 前4个关节（肩部、手臂）使用较大惯性，后3个关节（手腕）使用较小惯性
K_r = np.diag([0.3, 0.3, 0.3, 0.3, 0.18, 0.18, 0.18])

# 粘性摩擦系数矩阵 - 模拟关节的粘性阻尼
# 用于补偿实际系统与名义模型之间的动力学差异
K_l = np.diag([75.0, 75.0, 75.0, 75.0, 40.0, 40.0, 40.0])

# 位置相关摩擦系数矩阵 - 补偿位置相关的摩擦和建模误差
K_lp = np.diag([5.0, 5.0, 5.0, 5.0, 4.0, 4.0, 4.0])

# PD控制器增益矩阵
# K_p: 位置比例增益，决定位置误差的响应强度
K_p = np.diag([100.0, 100.0, 100.0, 100.0, 50.0, 50.0, 50.0])
# K_d: 速度微分增益，提供阻尼，防止振荡
K_d = np.diag([3.0, 3.0, 3.0, 3.0, 2.0, 2.0, 2.0])

# 预计算的矩阵，提高计算效率
K_r_inv = np.linalg.inv(K_r)  # 惯性矩阵的逆
K_r_K_l = K_r @ K_l           # 惯性与粘性摩擦的乘积

# 控制周期（1kHz）
DT = 0.001

# ==================== 低通滤波器类 ====================

class LowPassFilter:
    """
    一阶低通滤波器
    用于平滑传感器数据，减少噪声对控制系统的影响
    """
    def __init__(self, alpha, initial_value):
        """
        初始化低通滤波器
        
        Args:
            alpha: 滤波系数 (0-1)，越小滤波效果越强但响应越慢
            initial_value: 初始值
        """
        self.alpha = alpha
        self.y = initial_value

    def filter(self, x):
        """
        执行低通滤波
        
        公式: y[n] = α * x[n] + (1-α) * y[n-1]
        
        Args:
            x: 输入信号
            
        Returns:
            滤波后的信号
        """
        self.y = self.alpha * x + (1 - self.alpha) * self.y
        return self.y

# ==================== 关节顺应性控制器类 ====================

class JointCompliantController:
    """
    关节空间顺应性控制器
    
    实现基于阻抗控制的机械臂控制算法，主要特点：
    1. 柔顺性：能够适应外部力的作用
    2. 精确性：通过PD控制确保位置精度
    3. 平滑性：通过在线轨迹生成确保运动平滑
    4. 鲁棒性：通过摩擦补偿提高控制鲁棒性
    """
    
    def __init__(self, command_queue):
        """
        初始化控制器
        
        Args:
            command_queue: 命令队列，用于接收外部控制命令
        """
        # ========== 状态变量初始化 ==========
        self.q_s = None      # 传感器读取的关节角度（展开后）
        self.q_d = None      # 期望的关节角度
        self.dq_d = None     # 期望的关节速度
        self.q_n = None      # 名义模型的关节角度
        self.dq_n = None     # 名义模型的关节速度
        self.tau_filter = None  # 力矩传感器低通滤波器
        self.gripper_pos = None # 夹爪位置
        self.command_queue = command_queue  # 命令队列

        # ========== 在线轨迹生成器(OTG)初始化 ==========
        self.last_command_time = None  # 上次接收命令的时间
        self.otg = None       # Ruckig轨迹生成器
        self.otg_inp = None   # 轨迹生成器输入参数
        self.otg_out = None   # 轨迹生成器输出参数
        self.otg_res = None   # 轨迹生成器结果状态

        # 数据记录（调试用）
        # self.data = []

    def control_callback(self, arm):
        """
        控制回调函数 - 控制器的核心逻辑
        
        该函数以1kHz频率被调用，实现实时控制
        
        控制流程：
        1. 初始化（仅第一次调用时执行）
        2. 读取传感器数据并进行预处理
        3. 检查并处理新的控制命令
        4. 更新在线轨迹生成器
        5. 计算任务空间控制力矩
        6. 更新名义动力学模型
        7. 计算摩擦补偿力矩
        8. 输出最终控制力矩
        
        Args:
            arm: TorqueControlledArm实例，提供机械臂状态和控制接口
            
        Returns:
            tuple: (控制力矩, 夹爪位置)
        """
        
        # ========== 第一次调用时的初始化 ==========
        if self.q_s is None:
            # 初始化状态变量为当前机械臂状态
            self.q_s = arm.q.copy()      # 传感器关节角度
            self.q_d = arm.q.copy()      # 期望关节角度
            self.dq_d = np.zeros_like(arm.q)  # 期望关节速度（初始为0）
            self.q_n = arm.q.copy()      # 名义模型关节角度
            self.dq_n = arm.dq.copy()    # 名义模型关节速度
            
            # 初始化力矩传感器低通滤波器
            self.tau_filter = LowPassFilter(ALPHA, arm.tau.copy())
            self.gripper_pos = arm.gripper_pos  # 当前夹爪位置

            # ========== 初始化在线轨迹生成器 ==========
            self.last_command_time = time.time()
            self.otg = Ruckig(arm.actuator_count, DT)  # 创建轨迹生成器
            self.otg_inp = InputParameter(arm.actuator_count)  # 输入参数
            self.otg_out = OutputParameter(arm.actuator_count) # 输出参数
            
            # 设置关节速度和加速度限制
            # 前4个关节：最大速度80°/s，最大加速度240°/s²
            # 后3个关节：最大速度140°/s，最大加速度450°/s²
            self.otg_inp.max_velocity = 4 * [math.radians(80)] + 3 * [math.radians(140)]
            self.otg_inp.max_acceleration = 4 * [math.radians(240)] + 3 * [math.radians(450)]
            
            # 设置初始状态
            self.otg_inp.current_position = arm.q.copy()
            self.otg_inp.current_velocity = arm.dq.copy()
            self.otg_inp.target_position = arm.q.copy()  # 初始目标为当前位置
            self.otg_inp.target_velocity = np.zeros(arm.actuator_count)  # 目标速度为0
            self.otg_res = Result.Finished  # 初始状态为完成

        # ========== 传感器数据读取和预处理 ==========
        
        # 处理关节角度的周期性（-π到π的展开）
        # 这确保角度连续性，避免在±π边界处的跳跃
        self.q_s = self.q_s + np.mod(arm.q - self.q_s + np.pi, 2 * np.pi) - np.pi
        
        dq_s = arm.dq.copy()    # 关节速度
        tau_s = arm.tau.copy()  # 关节力矩（原始）
        tau_s_f = self.tau_filter.filter(tau_s)  # 滤波后的关节力矩

        # ========== 命令处理 ==========
        
        # 检查是否有新的控制命令
        if not self.command_queue.empty():
            qpos, self.gripper_pos = self.command_queue.get()  # 获取新命令
            self.last_command_time = time.time()  # 记录命令时间
            
            # 将目标位置转换为展开的角度表示
            qpos = self.q_s + np.mod(qpos - self.q_s + np.pi, 2 * np.pi) - np.pi
            
            # 设置新的轨迹目标
            self.otg_inp.target_position = qpos
            self.otg_res = Result.Working  # 开始生成新轨迹

        # ========== 安全机制：命令流中断检测 ==========
        
        # 如果超过2.5个控制周期没有收到新命令，则保持当前位置
        # 这防止了通信中断时机械臂的异常行为
        if time.time() - self.last_command_time > 2.5 * POLICY_CONTROL_PERIOD:
            self.otg_inp.target_position = self.otg_out.new_position
            self.otg_res = Result.Working

        # ========== 在线轨迹生成更新 ==========
        
        # 如果轨迹生成器正在工作，则更新轨迹
        if self.otg_res == Result.Working:
            self.otg_res = self.otg.update(self.otg_inp, self.otg_out)  # 计算下一个轨迹点
            self.otg_out.pass_to_input(self.otg_inp)  # 更新输入状态
            
            # 更新期望的位置和速度
            self.q_d[:] = self.otg_out.new_position
            self.dq_d[:] = self.otg_out.new_velocity

        # 数据记录（调试用）
        # self.data.append({
        #     'timestamp': time.time(),
        #     'q_s': self.q_s.tolist(),
        #     'dq_s': dq_s.tolist(),
        #     'q_d': self.q_d.tolist(),
        #     'dq_d': self.dq_d.tolist(),
        #     'target_position': self.otg_inp.target_position,
        #     'target_velocity': self.otg_inp.target_velocity,
        #     'new_position': self.otg_out.new_position,
        #     'new_velocity': self.otg_out.new_velocity,
        # })

        # ========== 任务空间控制力矩计算 ==========
        
        g = arm.gravity()  # 获取重力补偿力矩
        
        # PD控制器：计算完成任务所需的力矩
        # tau_task = -Kp*(q_n - q_d) - Kd*(dq_n - dq_d) + g
        # 其中：
        # - 第一项：位置误差的比例控制
        # - 第二项：速度误差的微分控制  
        # - 第三项：重力补偿
        tau_task = -K_p @ (self.q_n - self.q_d) - K_d @ (self.dq_n - self.dq_d) + g

        # ========== 名义动力学模型更新 ==========
        
        # 名义动力学方程：K_r * ddq_n = tau_task - tau_s_f
        # 这里假设名义模型是一个简单的惯性系统
        ddq_n = K_r_inv @ (tau_task - tau_s_f)  # 名义加速度
        
        # 数值积分更新名义状态
        self.dq_n += ddq_n * DT  # 速度积分
        self.q_n += self.dq_n * DT  # 位置积分

        # ========== 摩擦补偿力矩计算 ==========
        
        # 摩擦补偿用于减少实际系统与名义模型之间的差异
        # tau_f = K_r * K_l * ((dq_n - dq_s) + K_lp * (q_n - q_s))
        # 其中：
        # - (dq_n - dq_s)：速度误差补偿（粘性摩擦）
        # - K_lp * (q_n - q_s)：位置误差补偿（静摩擦和建模误差）
        tau_f = K_r_K_l @ ((self.dq_n - dq_s) + K_lp @ (self.q_n - self.q_s))

        # ========== 最终控制力矩输出 ==========
        
        # 总控制力矩 = 任务力矩 + 摩擦补偿力矩
        tau_c = tau_task + tau_f

        return tau_c, self.gripper_pos

# ==================== 测试和演示函数 ====================

def command_loop_retract(command_queue, stop_event):
    """
    收缩位置命令循环
    
    将机械臂移动到预定义的收缩位置，用于初始化或安全停止
    
    Args:
        command_queue: 命令队列
        stop_event: 停止事件
    """
    # 预定义的关节角度（收缩位置）
    # qpos = np.array([0.0, 0.26179939, 3.14159265, -2.26892803, 0.0, 0.95993109, 1.57079633])  # Home位置
    qpos = np.array([0.0, -0.34906585, 3.14159265, -2.54818071, 0.0, -0.87266463, 1.57079633])  # 收缩位置
    gripper_pos = 0  # 夹爪打开
    
    while not stop_event.is_set():
        command_queue.put((qpos, gripper_pos))
        time.sleep(POLICY_CONTROL_PERIOD)  # 注意：这不是精确的定时

def command_loop_circle(arm, command_queue, stop_event):
    """
    圆形轨迹命令循环
    
    让机械臂末端执行圆形轨迹运动，用于测试控制器性能
    
    Args:
        arm: 机械臂实例
        command_queue: 命令队列
        stop_event: 停止事件
    """
    from ik_solver import IKSolver
    
    # 初始化逆运动学求解器
    ik_solver = IKSolver(ee_offset=0.12)
    
    # 设置末端姿态（四元数表示）
    quat = np.array([0.707, 0.707, 0.0, 0.0])  # (x, y, z, w)
    
    # 圆形轨迹参数
    radius = 0.1  # 半径10cm
    num_points = 30  # 轨迹点数量
    center = np.array([0.45, 0.0, 0.2])  # 圆心位置
    
    # 生成圆形轨迹点
    t = np.linspace(0, 2 * np.pi, num_points)
    x = radius * np.cos(t)
    y = radius * np.sin(t)
    z = np.zeros(num_points)
    points = np.column_stack((x, y, z))
    points += center
    
    gripper_pos = 0  # 夹爪状态
    
    while not stop_event.is_set():
        for pos in points:
            # 通过逆运动学计算关节角度
            qpos = ik_solver.solve(pos, quat, arm.q)
            command_queue.put((qpos, gripper_pos))
            time.sleep(POLICY_CONTROL_PERIOD)  # 注意：这不是精确的定时

# ==================== 主程序入口 ====================

if __name__ == '__main__':
    """
    主程序：演示控制器的使用方法
    
    创建机械臂实例、控制器和命令线程，然后启动实时控制
    """
    import queue
    import threading
    
    # 创建机械臂实例
    arm = TorqueControlledArm()
    
    # 创建命令队列和控制器
    command_queue = queue.Queue(1)  # 队列大小为1，确保使用最新命令
    controller = JointCompliantController(command_queue)
    
    # 创建停止事件和命令线程
    stop_event = threading.Event()
    
    # 选择命令类型：收缩位置或圆形轨迹
    thread = threading.Thread(target=command_loop_retract, args=(command_queue, stop_event), daemon=True)
    # thread = threading.Thread(target=command_loop_circle, args=(arm, command_queue, stop_event), daemon=True)
    
    thread.start()
    
    # 初始化并启动实时控制
    arm.init_cyclic(controller.control_callback)
    
    try:
        # 保持控制循环运行
        while arm.cyclic_running:
            time.sleep(0.01)
    except KeyboardInterrupt:
        # 优雅关闭
        print("正在停止控制器...")
        stop_event.set()  # 停止命令线程
        thread.join()     # 等待线程结束
        time.sleep(0.5)   # 等待机械臂停止运动
        arm.stop_cyclic() # 停止实时控制
        arm.disconnect()  # 断开连接
        
        # 保存调试数据（可选）
        # import pickle
        # output_path = 'controller-states.pkl'
        # with open(output_path, 'wb') as f:
        #     pickle.dump(controller.data, f)
        # print(f'数据已保存到 {output_path}')
