# Author: Jimmy Wu
# Date: October 2024
# 中文注释版本：详细解释底盘控制原理和实现
#
# 重要安全提示：移动底盘配备强力电机，操作时需要密切监控
# 特别是在修改代码时，建议从低速开始测试，并将底盘侧翻使脚轮离地
# 使用转矩电流限制来限制电机的输出转矩
#
# 开发建议的额外安全措施：
# - 操作机器人时始终使用使能设备
# - 在机器人操作区域安装安全屏障
# - 在机器人上安装物理急停按钮
# - 在机器人上安装碰撞传感器
#
# 参考资料：
# - https://github.com/google/powered-caster-vehicle

# 设置Phoenix 6使用硬件而不是仿真
import os
os.environ['CTR_TARGET'] = 'Hardware'  # pylint: disable=wrong-import-position

import math
import queue
import threading
import time
from enum import Enum
import numpy as np
import phoenix6
from phoenix6 import configs, controls, hardware
from ruckig import InputParameter, OutputParameter, Result, Ruckig, ControlInterface
from threadpoolctl import threadpool_limits
from constants import h_x, h_y, ENCODER_MAGNET_OFFSETS
from constants import POLICY_CONTROL_PERIOD
from utils import create_pid_file

################################################################################
# 系统参数定义
################################################################################

# 底盘控制参数
CONTROL_FREQ = 250                   # 控制频率：250 Hz
CONTROL_PERIOD = 1.0 / CONTROL_FREQ  # 控制周期：4 ms
NUM_CASTERS = 4                      # 脚轮数量：4个

# 脚轮物理参数
b_x = -0.014008                      # 脚轮偏移量 (m) - 前后方向
b_y = -0.003753                      # 脚轮偏移量 (m) - 左右方向  
r = 0.0508                           # 轮子半径 (m)

# 传动比参数
N_s = 32.0 / 15.0 * 60.0 / 10.0      # 转向齿轮比
N_r1 = 50.0 / 14.0                   # 驱动齿轮比（第一级）
N_r2 = 19.0 / 25.0                   # 驱动齿轮比（第二级）
N_w = 45.0 / 15.0                    # 轮子齿轮比
N_r1_r2_w = N_r1 * N_r2 * N_w        # 组合驱动齿轮比
N_s_r2_w = N_s * N_r2 * N_w          # 组合转向齿轮比
TWO_PI = 2 * math.pi                 # 2π常数

################################################################################
# 电机类：封装单个TalonFX电机的控制逻辑
################################################################################

class Motor:
    """
    电机控制类
    
    功能：
    - 初始化TalonFX电机配置
    - 提供位置和速度读取接口
    - 提供速度控制和停止接口
    - 根据电机编号自动识别转向/驱动电机
    """
    
    def __init__(self, num):
        """
        初始化电机
        
        Args:
            num: 电机编号（奇数为转向电机，偶数为驱动电机）
        """
        self.num = num
        self.is_steer = num % 2 != 0  # 奇数编号为转向电机
        self.fx = hardware.TalonFX(self.num)
        
        # 验证Phoenix Pro许可证（FOC控制需要）
        assert self.fx.get_is_pro_licensed(), "必须有Phoenix Pro许可证才能使用FOC控制"
        
        # 重置电机位置
        self.fx.set_position(0)
        fx_cfg = configs.TalonFXConfiguration()

        # 第一个CAN设备需要等待CAN总线就绪并检查电压
        if self.num == 1:
            time.sleep(0.2)  # 等待CAN总线就绪
            supply_voltage = self.fx.get_supply_voltage().value
            print(f'电机供电电压: {supply_voltage:.2f} V')
            if supply_voltage < 11.5:
                raise Exception('电机供电电压过低，请给电池充电')

        # 状态信号配置
        self.position_signal = self.fx.get_position()
        self.velocity_signal = self.fx.get_velocity()
        self.status_signals = [self.position_signal, self.velocity_signal]

        # 控制请求配置
        self.velocity_request = controls.VelocityTorqueCurrentFOC(0)  # FOC速度控制
        self.neutral_request = controls.NeutralOut()                 # 空档输出

        # 速度控制增益
        fx_cfg.slot0.k_p = 5.0  # 比例增益
        # 转向电机设置微分增益防止脚轮抖动
        fx_cfg.slot0.k_d = 0.1 if self.is_steer else 0.0

        # 电流限制（硬地面无斜坡条件下）
        # 重要：这些值限制了底盘能产生的力，修改时需要非常小心
        torque_current_limit = 40 if self.is_steer else 10  # 转向40A，驱动10A
        fx_cfg.torque_current.peak_forward_torque_current = torque_current_limit
        fx_cfg.torque_current.peak_reverse_torque_current = -torque_current_limit

        # 禁用蜂鸣器
        fx_cfg.audio.beep_on_boot = False
        # fx_cfg.audio.beep_on_config = False  # 2024年10月尚未支持

        # 应用配置
        self.fx.configurator.apply(fx_cfg)

    def get_position(self):
        """获取电机位置（弧度）"""
        return TWO_PI * self.position_signal.value

    def get_velocity(self):
        """获取电机速度（弧度/秒）"""
        return TWO_PI * self.velocity_signal.value

    def set_velocity(self, velocity):
        """设置电机目标速度（弧度/秒）"""
        self.fx.set_control(self.velocity_request.with_velocity(velocity / TWO_PI))

    def set_neutral(self):
        """设置电机为空档状态"""
        self.fx.set_control(self.neutral_request)

################################################################################
# 脚轮类：封装单个脚轮的转向和驱动控制
################################################################################

class Caster:
    """
    脚轮控制类
    
    功能：
    - 管理一个脚轮的转向电机和驱动电机
    - 处理编码器数据和位置反馈
    - 提供脚轮级别的运动学计算
    - 实现转向角度和驱动速度的协调控制
    """
    
    def __init__(self, num):
        """
        初始化脚轮
        
        Args:
            num: 脚轮编号（1-4）
        """
        self.num = num
        # 脚轮包含两个电机：转向电机（奇数ID）和驱动电机（偶数ID）
        self.steer_motor = Motor(2 * self.num - 1)  # 转向电机
        self.drive_motor = Motor(2 * self.num)      # 驱动电机
        
        # CANcoder绝对编码器用于转向角度反馈
        self.cancoder = hardware.CANcoder(self.num)
        self.cancoder_cfg = configs.CANcoderConfiguration()

        # 状态信号配置
        self.steer_position_signal = self.cancoder.get_absolute_position()
        self.steer_velocity_signal = self.cancoder.get_velocity()
        # 合并所有状态信号
        self.status_signals = self.steer_motor.status_signals + self.drive_motor.status_signals
        self.status_signals.extend([self.steer_position_signal, self.steer_velocity_signal])

        # 编码器磁体偏移校准
        self.cancoder_cfg.magnet_sensor.magnet_offset = ENCODER_MAGNET_OFFSETS[self.num - 1]
        self.cancoder.configurator.apply(self.cancoder_cfg)

    def get_steer_position(self):
        """获取转向位置（弧度）"""
        return TWO_PI * self.steer_position_signal.value

    def get_steer_velocity(self):
        """获取转向速度（弧度/秒）"""
        return TWO_PI * self.steer_velocity_signal.value

    def get_positions(self):
        """
        获取脚轮位置
        
        Returns:
            tuple: (转向位置, 驱动位置) 单位：弧度
            
        注意：驱动位置通过差分计算得出，考虑了转向运动对驱动的影响
        """
        steer_motor_pos = self.steer_motor.get_position()
        drive_motor_pos = self.drive_motor.get_position()
        
        # 转向位置直接从编码器读取（更精确）
        steer_pos = self.get_steer_position()
        
        # 驱动位置需要补偿转向运动的影响
        # 这是因为转向时会带动驱动轮转动
        drive_pos = steer_motor_pos / N_s_r2_w - drive_motor_pos / N_r1_r2_w
        
        return steer_pos, drive_pos

    def get_velocities(self):
        """
        获取脚轮速度
        
        Returns:
            tuple: (转向速度, 驱动速度) 单位：弧度/秒
        """
        steer_motor_vel = self.steer_motor.get_velocity()
        drive_motor_vel = self.drive_motor.get_velocity()
        
        # 转向速度通过齿轮比计算
        steer_vel = steer_motor_vel / N_s
        # 注意：直接从编码器读取的转向速度噪声很大，所以使用电机速度
        
        # 驱动速度需要补偿转向运动的影响
        drive_vel = steer_motor_vel / N_s_r2_w - drive_motor_vel / N_r1_r2_w
        
        return steer_vel, drive_vel

    def set_velocities(self, steer_vel, drive_vel):
        """
        设置脚轮目标速度
        
        Args:
            steer_vel: 转向速度（弧度/秒）
            drive_vel: 驱动速度（弧度/秒）
            
        注意：需要考虑转向和驱动的耦合关系
        """
        # 转向电机速度直接按齿轮比缩放
        self.steer_motor.set_velocity(N_s * steer_vel)
        
        # 驱动电机速度需要补偿转向运动
        # 当脚轮转向时，会带动驱动轮转动，需要在驱动电机上补偿这个影响
        self.drive_motor.set_velocity(N_r1 * steer_vel - N_r1_r2_w * drive_vel)

    def set_neutral(self):
        """设置脚轮为空档状态"""
        self.steer_motor.set_neutral()
        self.drive_motor.set_neutral()

################################################################################
# 命令类型枚举
################################################################################

class CommandType(Enum):
    """控制命令类型"""
    POSITION = 'position'  # 位置控制
    VELOCITY = 'velocity'  # 速度控制

class FrameType(Enum):
    """坐标系类型（仅用于速度命令）"""
    GLOBAL = 'global'  # 全局坐标系
    LOCAL = 'local'    # 本体坐标系

################################################################################
# 底盘控制类：实现完整的底盘运动控制
################################################################################

class Vehicle:
    """
    底盘控制类
    
    功能：
    - 管理4个脚轮的协调运动
    - 实现运动学正逆解算
    - 提供平滑的轨迹生成
    - 实现实时控制循环
    - 处理安全保护和状态监控
    """
    
    def __init__(self, max_vel=(0.5, 0.5, 1.57), max_accel=(0.25, 0.25, 0.79)):
        """
        初始化底盘控制器
        
        Args:
            max_vel: 最大速度限制 (vx, vy, wz) 单位：m/s, m/s, rad/s
            max_accel: 最大加速度限制 (ax, ay, az) 单位：m/s², m/s², rad/s²
        """
        self.max_vel = np.array(max_vel)
        self.max_accel = np.array(max_accel)

        # 使用PID文件确保单实例运行
        create_pid_file('tidybot2-base-controller')

        # 初始化脚轮
        self.casters = [Caster(num) for num in range(1, NUM_CASTERS + 1)]

        # CAN总线更新频率设置
        self.status_signals = [signal for caster in self.casters for signal in caster.status_signals]
        phoenix6.BaseStatusSignal.set_update_frequency_for_all(CONTROL_FREQ, self.status_signals)

        # 关节空间变量（8个电机：4个转向 + 4个驱动）
        num_motors = 2 * NUM_CASTERS
        self.q = np.zeros(num_motors)    # 关节位置
        self.dq = np.zeros(num_motors)   # 关节速度
        self.tau = np.zeros(num_motors)  # 关节力矩

        # 操作空间变量（全局坐标系：x, y, θ）
        num_dofs = 3
        self.x = np.zeros(num_dofs)   # 底盘位姿 [x, y, θ]
        self.dx = np.zeros(num_dofs)  # 底盘速度 [vx, vy, wz]

        # 运动学矩阵初始化
        self._init_kinematics_matrices(num_motors, num_dofs)
        
        # 在线轨迹生成器（OTG）初始化
        self._init_trajectory_generator(num_dofs)
        
        # 控制循环初始化
        self._init_control_loop()

    def _init_kinematics_matrices(self, num_motors, num_dofs):
        """初始化运动学矩阵"""
        
        # C矩阵：将操作空间速度映射到关节速度
        # dq = C * dx_local （其中dx_local是本体坐标系下的速度）
        self.C = np.zeros((num_motors, num_dofs))
        self.C_steer = self.C[::2]  # 转向电机行（偶数索引）
        self.C_drive = self.C[1::2] # 驱动电机行（奇数索引）

        # C_p矩阵：将操作空间速度映射到接触点处的轮子速度
        self.C_p = np.zeros((num_motors, num_dofs))
        self.C_p_steer = self.C_p[::2]
        self.C_p_drive = self.C_p[1::2]
        # 转向轮在接触点处的速度约束
        self.C_p_steer[:, :2] = [1.0, 0.0]  # 转向轮只能沿x方向滚动
        self.C_p_drive[:, :2] = [0.0, 1.0]  # 驱动轮只能沿y方向滚动

        # C_pinv矩阵：C_p的广义逆矩阵，用于从关节速度计算操作空间速度
        self.C_pinv = np.zeros((num_motors, num_dofs))
        self.CpT_Cqinv = np.zeros((num_dofs, num_motors))
        self.CpT_Cqinv_steer = self.CpT_Cqinv[:, ::2]
        self.CpT_Cqinv_drive = self.CpT_Cqinv[:, 1::2]

    def _init_trajectory_generator(self, num_dofs):
        """初始化在线轨迹生成器"""
        
        # 注意：理想情况下应该使用极坐标耦合x和y轴
        self.otg = Ruckig(num_dofs, CONTROL_PERIOD)
        self.otg_inp = InputParameter(num_dofs)
        self.otg_out = OutputParameter(num_dofs)
        self.otg_res = Result.Working
        
        # 设置速度和加速度限制
        self.otg_inp.max_velocity = self.max_vel
        self.otg_inp.max_acceleration = self.max_accel

    def _init_control_loop(self):
        """初始化控制循环"""
        
        self.command_queue = queue.Queue(1)  # 命令队列（容量为1）
        self.control_loop_thread = threading.Thread(target=self.control_loop, daemon=True)
        self.control_loop_running = False

        # 调试相关（可选）
        # self.data = []
        # import redis
        # self.redis_client = redis.Redis()

    def update_state(self):
        """
        更新底盘状态
        
        功能：
        1. 读取所有传感器数据
        2. 计算运动学矩阵
        3. 更新里程计
        """
        
        # 更新所有状态信号（传感器数值）
        # 注意：信号延迟大约4ms
        phoenix6.BaseStatusSignal.refresh_all(self.status_signals)

        # 读取关节位置和速度
        for i, caster in enumerate(self.casters):
            self.q[2*i : 2*i + 2] = caster.get_positions()
            self.dq[2*i : 2*i + 2] = caster.get_velocities()

        # 提取转向角度并计算三角函数
        q_steer = self.q[::2]  # 转向角度
        s = np.sin(q_steer)    # sin(转向角)
        c = np.cos(q_steer)    # cos(转向角)

        # 更新C矩阵（运动学雅可比矩阵）
        # 转向电机的运动学关系
        self.C_steer[:, 0] = s / b_x                              # x方向
        self.C_steer[:, 1] = -c / b_x                             # y方向  
        self.C_steer[:, 2] = (-h_x*c - h_y*s) / b_x - 1.0        # 旋转

        # 驱动电机的运动学关系
        self.C_drive[:, 0] = c/r - b_y*s / (b_x*r)               # x方向
        self.C_drive[:, 1] = s/r + b_y*c / (b_x*r)               # y方向
        self.C_drive[:, 2] = (h_x*s - h_y*c) / r + b_y * (h_x*c + h_y*s) / (b_x*r)  # 旋转

        # 更新C_p矩阵（接触点运动学）
        self.C_p_steer[:, 2] = -b_x*s - b_y*c - h_y              # 转向轮旋转分量
        self.C_p_drive[:, 2] = b_x*c - b_y*s + h_x               # 驱动轮旋转分量

        # 更新C_qp^#矩阵（用于里程计计算）
        self.CpT_Cqinv_steer[0] = b_x*s + b_y*c
        self.CpT_Cqinv_steer[1] = -b_x*c + b_y*s  
        self.CpT_Cqinv_steer[2] = b_x * (-h_x*c - h_y*s - b_x) + b_y * (h_x*s - h_y*c - b_y)
        self.CpT_Cqinv_drive[0] = r * c
        self.CpT_Cqinv_drive[1] = r * s
        self.CpT_Cqinv_drive[2] = r * (h_x*s - h_y*c - b_y)
        
        # 计算广义逆矩阵（限制BLAS线程数防止CPU过度使用）
        with threadpool_limits(limits=1, user_api='blas'):
            self.C_pinv = np.linalg.solve(self.C_p.T @ self.C_p, self.CpT_Cqinv)

        # 里程计更新
        dx_local = self.C_pinv @ self.dq  # 本体坐标系下的速度
        
        # 使用中点法进行数值积分，提高精度
        theta_avg = self.x[2] + 0.5 * dx_local[2] * CONTROL_PERIOD
        
        # 本体坐标系到全局坐标系的旋转矩阵
        R = np.array([
            [math.cos(theta_avg), -math.sin(theta_avg), 0.0],
            [math.sin(theta_avg), math.cos(theta_avg), 0.0],
            [0.0, 0.0, 1.0]
        ])
        
        # 转换到全局坐标系并更新位姿
        self.dx = R @ dx_local
        self.x += self.dx * CONTROL_PERIOD

    def start_control(self):
        """启动控制循环"""
        if self.control_loop_thread is None:
            print('要启动新的控制循环，请创建Vehicle的新实例')
            return
        self.control_loop_running = True
        self.control_loop_thread.start()

    def stop_control(self):
        """停止控制循环"""
        self.control_loop_running = False
        self.control_loop_thread.join()
        self.control_loop_thread = None

    def control_loop(self):
        """
        主控制循环
        
        功能：
        1. 维持250Hz的控制频率
        2. 处理命令队列中的新命令
        3. 执行轨迹生成和运动控制
        4. 实现安全保护机制
        """
        
        # 尝试设置实时调度策略（需要系统权限）
        try:
            os.sched_setscheduler(0, os.SCHED_FIFO, 
                                os.sched_param(os.sched_get_priority_max(os.SCHED_FIFO)))
        except PermissionError:
            print('无法设置实时调度策略，请编辑 /etc/security/limits.d/99-realtime.conf')

        # 控制循环变量初始化
        disable_motors = True           # 电机禁用标志
        last_command_time = time.time() # 上次命令时间
        last_step_time = time.time()    # 上次循环时间
        
        while self.control_loop_running:
            # 维持期望的控制频率（250Hz = 4ms周期）
            while time.time() - last_step_time < CONTROL_PERIOD:
                time.sleep(0.0001)  # 短暂休眠避免忙等待
                
            curr_time = time.time()
            step_time = curr_time - last_step_time
            last_step_time = curr_time
            
            # 监控控制循环时序
            if step_time > 0.005:  # 超过5ms发出警告
                print(f'警告：控制循环步进时间 {1000 * step_time:.3f} ms 在 {self.__class__.__name__}')

            # 更新底盘状态
            self.update_state()

            # 全局坐标系到本体坐标系的转换矩阵
            theta = self.x[2]
            R = np.array([
                [math.cos(theta), math.sin(theta), 0.0],
                [-math.sin(theta), math.cos(theta), 0.0],
                [0.0, 0.0, 1.0]
            ])

            # 检查新命令
            if not self.command_queue.empty():
                command = self.command_queue.get()
                last_command_time = time.time()
                target = command['target']

                # 速度控制命令
                if command['type'] == CommandType.VELOCITY:
                    # 如果是本体坐标系命令，转换到全局坐标系
                    if command['frame'] == FrameType.LOCAL:
                        target = R.T @ target
                    
                    # 设置轨迹生成器为速度控制模式
                    self.otg_inp.control_interface = ControlInterface.Velocity
                    self.otg_inp.target_velocity = np.clip(target, -self.max_vel, self.max_vel)

                # 位置控制命令
                elif command['type'] == CommandType.POSITION:
                    self.otg_inp.control_interface = ControlInterface.Position
                    self.otg_inp.target_position = target
                    self.otg_inp.target_velocity = np.zeros_like(self.dx)

                # 重置轨迹生成器状态
                self.otg_res = Result.Working
                disable_motors = False

            # 命令流中断保护：如果长时间没有新命令，保持当前位置
            if time.time() - last_command_time > 2.5 * POLICY_CONTROL_PERIOD:
                self.otg_inp.target_position = self.otg_out.new_position
                self.otg_inp.target_velocity = np.zeros_like(self.dx)
                self.otg_inp.current_velocity = self.dx  # 防止命令恢复时的突跳
                self.otg_res = Result.Working
                disable_motors = True

            # 脚轮翻转时减速保护
            # 注意：在低速时可以禁用此功能以获得更平滑的运动
            if np.max(np.abs(self.dq[::2])) > 12.56:  # 转向关节速度 > 720度/秒
                if self.otg_inp.control_interface == ControlInterface.Position:
                    self.otg_inp.target_position = self.otg_out.new_position
                elif self.otg_inp.control_interface == ControlInterface.Velocity:
                    self.otg_inp.target_velocity = np.zeros_like(self.dx)

            # 更新轨迹生成器
            if self.otg_res == Result.Working:
                self.otg_inp.current_position = self.x
                self.otg_res = self.otg.update(self.otg_inp, self.otg_out)
                self.otg_out.pass_to_input(self.otg_inp)

            # 电机控制
            if disable_motors:
                # 发送电机空档命令
                for i in range(NUM_CASTERS):
                    self.casters[i].set_neutral()
            else:
                # 发送使能信号到设备
                phoenix6.unmanaged.feed_enable(0.1)

                # 操作空间期望速度
                dx_d = self.otg_out.new_velocity
                dx_d_local = R @ dx_d  # 转换到本体坐标系

                # 计算关节空间期望速度
                dq_d = self.C @ dx_d_local

                # 发送电机速度命令
                for i in range(NUM_CASTERS):
                    self.casters[i].set_velocities(dq_d[2*i], dq_d[2*i + 1])

            # 调试数据记录（可选）
            # self.data.append({
            #     'timestamp': time.time(),
            #     'q': self.q.tolist(),
            #     'dq': self.dq.tolist(), 
            #     'x': self.x.tolist(),
            #     'dx': self.dx.tolist(),
            # })

    def _enqueue_command(self, command_type, target, frame=None):
        """
        将命令加入队列
        
        Args:
            command_type: 命令类型
            target: 目标值
            frame: 坐标系类型（可选）
        """
        if self.command_queue.full():
            print('警告：命令队列已满。控制循环是否在运行？')
        else:
            command = {'type': command_type, 'target': target}
            if frame is not None:
                command['frame'] = FrameType(frame)
            self.command_queue.put(command, block=False)

    def set_target_velocity(self, velocity, frame='global'):
        """
        设置目标速度
        
        Args:
            velocity: 速度向量 [vx, vy, wz]
            frame: 坐标系 ('global' 或 'local')
        """
        self._enqueue_command(CommandType.VELOCITY, velocity, frame)

    def set_target_position(self, position):
        """
        设置目标位置
        
        Args:
            position: 位置向量 [x, y, theta]
        """
        self._enqueue_command(CommandType.POSITION, position)

    def get_encoder_offsets(self):
        """
        获取编码器偏移量（用于标定）
        
        该函数用于标定脚轮转向编码器的零位偏移
        """
        offsets = []
        for caster in self.casters:
            # 读取当前配置
            caster.cancoder.configurator.refresh(caster.cancoder_cfg)
            curr_offset = caster.cancoder_cfg.magnet_sensor.magnet_offset
            
            # 等待位置更新
            caster.steer_position_signal.wait_for_update(0.1)
            curr_position = caster.cancoder.get_absolute_position().value
            
            # 计算新的偏移量
            offsets.append(f'{round(4096 * (curr_offset - curr_position))}.0 / 4096')
        
        print(f'ENCODER_MAGNET_OFFSETS = [{", ".join(offsets)}]')

################################################################################
# 主程序：用于测试和演示
################################################################################

if __name__ == '__main__':
    # 创建底盘控制器实例（降低最大速度用于安全测试）
    vehicle = Vehicle(max_vel=(0.25, 0.25, 0.79))
    
    # 可选：获取编码器偏移量并退出
    # vehicle.get_encoder_offsets(); exit()
    
    # 启动控制循环
    vehicle.start_control()
    
    try:
        # 测试运动：原地旋转
        for _ in range(50):
            vehicle.set_target_velocity(np.array([0.0, 0.0, 0.39]))  # 纯旋转
            # vehicle.set_target_velocity(np.array([0.25, 0.0, 0.0]))  # 纯前进
            # vehicle.set_target_position(np.array([0.5, 0.0, 0.0]))   # 位置控制
            
            print(f'底盘状态 - 位置: {vehicle.x} 速度: {vehicle.dx}')
            time.sleep(POLICY_CONTROL_PERIOD)  # 注意：时序不够精确
            
    finally:
        # 清理资源
        vehicle.stop_control()
        
        # 可选：保存调试数据
        # import pickle
        # output_path = 'controller-states.pkl'
        # with open(output_path, 'wb') as f:
        #     pickle.dump(vehicle.data, f)
        # print(f'数据已保存到 {output_path}') 