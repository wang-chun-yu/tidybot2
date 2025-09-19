#!/usr/bin/env python3
"""
TidyBot2 Android底盘遥控优化版

专门针对底盘控制需求优化的遥控实现：
- 双摇杆控制：移动摇杆 + 旋转摇杆
- 独立速度控制：移动速度滑动条 + 旋转速度滑动条
- 精确控制：手指离开摇杆时立即停止
- 实时反馈：显示当前速度和位置信息
- 安全保护：速度限制和超时保护

Author: AI Assistant
Date: September 2024
"""

import time
import json
import numpy as np
import threading
import queue
import base64
import cv2
from flask import Flask, render_template, request
from flask_socketio import SocketIO, emit
import logging
from collections import deque
import psutil
import gc
import math

from constants import POLICY_CONTROL_PERIOD
from policies import Policy

# 配置日志
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class BaseOptimizedAndroidTeleopServer:
    """专门针对底盘控制优化的Android遥控服务器"""
    
    def __init__(self, host='0.0.0.0', port=5000, debug=False):
        self.host = host
        self.port = port
        self.debug = debug
        
        # Flask应用设置（优化配置）
        self.app = Flask(__name__)
        self.app.config.update(
            SECRET_KEY='tidybot2_base_android_teleop_optimized',
            # 性能优化配置
            SEND_FILE_MAX_AGE_DEFAULT=31536000,  # 1年缓存
            MAX_CONTENT_LENGTH=16 * 1024 * 1024,  # 16MB最大请求
        )
        
        # SocketIO优化配置
        self.socketio = SocketIO(
            self.app, 
            cors_allowed_origins="*",
            logger=debug, 
            engineio_logger=debug,
            # 性能优化参数
            ping_timeout=60,
            ping_interval=25,
            max_http_buffer_size=1000000,  # 1MB缓冲区
            compression=True,  # 启用压缩
            async_mode='threading'  # 使用线程模式
        )
        
        # 底盘控制器
        self.controller = BaseOptimizedAndroidTeleopController()
        self.connected_clients = {}  # 使用字典存储客户端详细信息
        self.performance_monitor = PerformanceMonitor()
        
        # 数据队列（异步处理）
        self.message_queue = queue.Queue(maxsize=100)
        self.status_queue = queue.Queue(maxsize=50)
        
        # 启动后台处理线程
        self.start_background_threads()
        
        self.setup_routes()
        self.setup_socketio()
    
    def start_background_threads(self):
        """启动后台处理线程"""
        # 消息处理线程
        threading.Thread(target=self._message_processor, daemon=True).start()
        
        # 状态广播线程
        threading.Thread(target=self._status_broadcaster, daemon=True).start()
        
        # 性能监控线程
        threading.Thread(target=self._performance_monitor, daemon=True).start()
        
        # 垃圾回收优化线程
        threading.Thread(target=self._gc_optimizer, daemon=True).start()
    
    def _message_processor(self):
        """异步消息处理器"""
        while True:
            try:
                message_data = self.message_queue.get(timeout=1.0)
                self._process_message_async(message_data)
            except queue.Empty:
                continue
            except Exception as e:
                logger.error(f'Message processor error: {e}')
    
    def _status_broadcaster(self):
        """状态广播器 - 实时发送底盘状态"""
        while True:
            try:
                # 获取底盘状态
                status = self.controller.get_status()
                if status:
                    self.socketio.emit('base_status', status)
                time.sleep(0.05)  # 20Hz更新频率
            except Exception as e:
                logger.error(f'Status broadcaster error: {e}')
                time.sleep(0.1)
    
    def _performance_monitor(self):
        """性能监控线程"""
        while True:
            try:
                stats = self.performance_monitor.get_stats()
                self.socketio.emit('performance_stats', stats)
                time.sleep(5)  # 每5秒更新一次
            except Exception as e:
                logger.error(f'Performance monitor error: {e}')
                time.sleep(5)
    
    def _gc_optimizer(self):
        """垃圾回收优化器"""
        while True:
            try:
                time.sleep(30)
                collected = gc.collect()
                if collected > 0:
                    logger.debug(f'GC collected {collected} objects')
            except Exception as e:
                logger.error(f'GC optimizer error: {e}')
    
    def setup_routes(self):
        """设置优化的Flask路由"""
        
        @self.app.route('/')
        def index():
            return render_template('android_base_teleop.html')
        
        @self.app.route('/base')
        def base_control():
            return render_template('android_base_teleop.html')
        
        @self.app.route('/debug')
        def debug():
            return render_template('android_base_debug.html')
        
        @self.app.route('/status')
        def status():
            return {
                'status': 'running',
                'clients': len(self.connected_clients),
                'base_status': self.controller.get_detailed_status(),
                'performance': self.performance_monitor.get_stats(),
                'timestamp': time.time()
            }
        
        @self.app.route('/health')
        def health():
            return {'status': 'healthy', 'timestamp': time.time()}
        
        # 静态文件优化
        @self.app.after_request
        def after_request(response):
            response.headers['Cache-Control'] = 'public, max-age=31536000'
            response.headers['Vary'] = 'Accept-Encoding'
            return response
    
    def setup_socketio(self):
        """设置优化的SocketIO事件处理"""
        
        @self.socketio.on('connect')
        def handle_connect():
            client_id = request.sid
            client_info = {
                'id': client_id,
                'connect_time': time.time(),
                'last_activity': time.time(),
                'message_count': 0,
                'user_agent': request.headers.get('User-Agent', ''),
                'ip': request.remote_addr
            }
            
            self.connected_clients[client_id] = client_info
            logger.info(f'Base teleop client {client_id} connected from {client_info["ip"]}')
            
            # 发送底盘控制配置
            emit('base_config', {
                'client_id': client_id,
                'server_time': time.time(),
                'max_linear_speed': 0.5,  # m/s
                'max_angular_speed': 1.57,  # rad/s
                'control_frequency': 20,  # Hz
                'deadzone': 0.1,
                'optimization_level': 'high'
            })
        
        @self.socketio.on('disconnect')
        def handle_disconnect():
            client_id = request.sid
            if client_id in self.connected_clients:
                client_info = self.connected_clients[client_id]
                session_duration = time.time() - client_info['connect_time']
                logger.info(f'Base teleop client {client_id} disconnected after {session_duration:.1f}s')
                del self.connected_clients[client_id]
            
            # 停止底盘运动
            self.controller.emergency_stop()
            
            # 如果没有客户端，进入省电模式
            if len(self.connected_clients) == 0:
                self.controller.enter_power_save_mode()
        
        @self.socketio.on('base_control')
        def handle_base_control(data):
            """处理底盘控制命令"""
            client_id = request.sid
            
            # 更新客户端活动时间
            if client_id in self.connected_clients:
                self.connected_clients[client_id]['last_activity'] = time.time()
                self.connected_clients[client_id]['message_count'] += 1
            
            # 异步处理底盘控制消息
            try:
                self.message_queue.put_nowait({
                    'client_id': client_id,
                    'type': 'base_control',
                    'data': data,
                    'timestamp': time.time()
                })
            except queue.Full:
                logger.warning(f'Base control queue full, dropping message from {client_id}')
        
        @self.socketio.on('base_stop')
        def handle_base_stop():
            """处理底盘停止命令"""
            self.controller.emergency_stop()
            emit('base_stopped', {'timestamp': time.time()})
        
        @self.socketio.on('episode_control')
        def handle_episode_control(data):
            """处理操作控制命令"""
            action = data.get('action')
            if action == 'start':
                self.controller.start_episode()
            elif action == 'stop':
                self.controller.end_episode()
            elif action == 'reset':
                self.controller.reset_environment()
            
            emit('episode_status', {
                'active': self.controller.episode_active,
                'timestamp': time.time()
            })
        
        @self.socketio.on('speed_config')
        def handle_speed_config(data):
            """处理速度配置"""
            linear_scale = data.get('linear_scale', 1.0)
            angular_scale = data.get('angular_scale', 1.0)
            
            self.controller.set_speed_scales(linear_scale, angular_scale)
            
            emit('speed_config_ack', {
                'linear_scale': linear_scale,
                'angular_scale': angular_scale,
                'timestamp': time.time()
            })
    
    def _process_message_async(self, message_data):
        """异步处理消息"""
        client_id = message_data['client_id']
        msg_type = message_data['type']
        data = message_data['data']
        
        try:
            if msg_type == 'base_control':
                self.controller.process_base_control(data)
                
        except Exception as e:
            logger.error(f'Error processing async message: {e}')
    
    def run(self):
        """启动优化的服务器"""
        logger.info(f'Starting Base Optimized Android Teleop Server on {self.host}:{self.port}')
        self.socketio.run(
            self.app, 
            host=self.host, 
            port=self.port,
            debug=self.debug,
            allow_unsafe_werkzeug=True,
            use_reloader=False
        )

class BaseOptimizedAndroidTeleopController:
    """专门针对底盘控制优化的Android遥控控制器"""
    
    def __init__(self):
        # 底盘控制状态
        self.episode_active = False
        self.last_control_time = time.time()
        self.power_save_mode = False
        
        # 底盘运动参数（基于TidyBot2实际参数）
        self.max_linear_speed = 0.5   # m/s (底盘最大线速度)
        self.max_angular_speed = 1.57  # rad/s (底盘最大角速度)
        self.max_acceleration = 0.25   # m/s² (最大加速度)
        self.max_angular_acceleration = 0.79  # rad/s² (最大角加速度)
        
        # 用户可调节的速度比例
        self.linear_speed_scale = 0.8   # 默认80%线速度
        self.angular_speed_scale = 0.8  # 默认80%角速度
        
        # 当前底盘状态（高效的数据结构）
        self.base_state = {
            'position': np.zeros(3, dtype=np.float32),  # [x, y, θ]
            'velocity': np.zeros(3, dtype=np.float32),  # [vx, vy, wz]
            'target_velocity': np.zeros(3, dtype=np.float32),  # 目标速度
            'joystick_input': np.zeros(3, dtype=np.float32),   # 摇杆输入
        }
        
        # 控制输入滤波器（减少抖动）
        self.input_filter = AdaptiveFilter(alpha=0.8, adaptation_rate=0.1)
        self.velocity_filter = AdaptiveFilter(alpha=0.7, adaptation_rate=0.05)
        
        # 控制参数
        self.deadzone = 0.1           # 摇杆死区
        self.update_rate = 20         # Hz
        self.timeout_threshold = 0.5  # 超时阈值（秒）
        
        # 数据缓存（优化性能）
        self.control_cache = {}
        self.cache_timeout = 0.02  # 20ms缓存
        
        # 安全参数
        self.emergency_stop_active = False
        self.max_control_change = 0.2  # 单次最大控制变化
        
        logger.info('Base Optimized Android Teleop Controller initialized')
    
    def enter_power_save_mode(self):
        """进入省电模式"""
        self.power_save_mode = True
        self.update_rate = 5  # 降低更新频率
        self.emergency_stop()
        logger.info('Base controller entered power save mode')
    
    def exit_power_save_mode(self):
        """退出省电模式"""
        self.power_save_mode = False
        self.update_rate = 20  # 恢复正常频率
        self.emergency_stop_active = False
        logger.info('Base controller exited power save mode')
    
    def set_speed_scales(self, linear_scale, angular_scale):
        """设置速度比例"""
        self.linear_speed_scale = max(0.1, min(1.0, linear_scale))
        self.angular_speed_scale = max(0.1, min(1.0, angular_scale))
        logger.info(f'Speed scales updated: linear={self.linear_speed_scale:.2f}, angular={self.angular_speed_scale:.2f}')
    
    def process_base_control(self, data):
        """处理底盘控制数据"""
        if not self.episode_active or self.emergency_stop_active:
            return
        
        current_time = time.time()
        
        # 数据缓存检查
        data_hash = hash(str(data))
        if (data_hash in self.control_cache and 
            current_time - self.control_cache[data_hash]['timestamp'] < self.cache_timeout):
            return  # 使用缓存数据，避免重复处理
        
        try:
            self.last_control_time = current_time
            
            # 提取控制输入
            linear_x = self._apply_deadzone(data.get('linear_x', 0.0))
            linear_y = self._apply_deadzone(data.get('linear_y', 0.0))
            
            # 旋转控制：处理方向值（-1, 0, 1）
            angular_direction = data.get('angular_z', 0.0)
            # 将方向值转换为归一化的角速度控制
            if angular_direction > 0:
                angular_z = 1.0  # 顺时针最大速度
            elif angular_direction < 0:
                angular_z = -1.0  # 逆时针最大速度
            else:
                angular_z = 0.0  # 停止旋转
            
            # 应用死区（对于按钮控制，死区主要用于线性移动）
            angular_z = self._apply_deadzone(angular_z)
            
            # 创建输入向量
            raw_input = np.array([linear_x, linear_y, angular_z], dtype=np.float32)
            
            # 应用输入滤波（对旋转按钮使用不同的滤波策略）
            filtered_input = self.input_filter.update(raw_input)
            
            # 对于按钮控制的旋转，保持原始的方向值，避免过度滤波
            if abs(angular_direction) > 0:
                filtered_input[2] = angular_z  # 保持按钮的明确方向
            
            # 限制输入变化率（安全保护）
            input_change = np.linalg.norm(filtered_input - self.base_state['joystick_input'])
            if input_change > self.max_control_change:
                # 限制变化率
                direction = (filtered_input - self.base_state['joystick_input'])
                direction_norm = np.linalg.norm(direction)
                if direction_norm > 0:
                    direction = direction / direction_norm
                    filtered_input = self.base_state['joystick_input'] + direction * self.max_control_change
            
            # 更新摇杆输入（存储原始方向值用于状态显示）
            self.base_state['joystick_input'] = np.array([
                filtered_input[0], 
                filtered_input[1], 
                float(angular_direction)  # 存储原始方向值
            ], dtype=np.float32)
            
            # 计算目标速度（考虑用户设置的速度比例）
            target_velocity = np.array([
                filtered_input[0] * self.max_linear_speed * self.linear_speed_scale,   # vx
                filtered_input[1] * self.max_linear_speed * self.linear_speed_scale,   # vy
                filtered_input[2] * self.max_angular_speed * self.angular_speed_scale  # wz
            ], dtype=np.float32)
            
            # 应用速度滤波（平滑控制）
            self.base_state['target_velocity'] = self.velocity_filter.update(target_velocity)
            
            # 更新底盘位置（积分）
            dt = 0.05  # 假设50ms更新周期
            self._update_base_position(dt)
            
            # 更新缓存
            self.control_cache[data_hash] = {
                'timestamp': current_time,
                'processed': True
            }
            
            # 限制缓存大小
            if len(self.control_cache) > 50:
                # 清理旧缓存
                old_keys = [k for k, v in self.control_cache.items() 
                           if current_time - v['timestamp'] > self.cache_timeout * 2]
                for key in old_keys:
                    del self.control_cache[key]
                    
        except Exception as e:
            logger.error(f'Error processing base control data: {e}')
    
    def _apply_deadzone(self, value):
        """应用死区处理"""
        if abs(value) < self.deadzone:
            return 0.0
        sign = 1 if value > 0 else -1
        return sign * (abs(value) - self.deadzone) / (1.0 - self.deadzone)
    
    def _update_base_position(self, dt):
        """更新底盘位置（简化的运动学积分）"""
        # 当前速度（考虑加速度限制）
        velocity_diff = self.base_state['target_velocity'] - self.base_state['velocity']
        max_vel_change = np.array([
            self.max_acceleration * dt,
            self.max_acceleration * dt,
            self.max_angular_acceleration * dt
        ])
        
        # 限制速度变化率
        velocity_diff = np.clip(velocity_diff, -max_vel_change, max_vel_change)
        self.base_state['velocity'] += velocity_diff
        
        # 更新位置（全局坐标系）
        theta = self.base_state['position'][2]
        vx_global = (self.base_state['velocity'][0] * np.cos(theta) - 
                     self.base_state['velocity'][1] * np.sin(theta))
        vy_global = (self.base_state['velocity'][0] * np.sin(theta) + 
                     self.base_state['velocity'][1] * np.cos(theta))
        
        self.base_state['position'][0] += vx_global * dt
        self.base_state['position'][1] += vy_global * dt
        self.base_state['position'][2] += self.base_state['velocity'][2] * dt
        
        # 角度归一化
        self.base_state['position'][2] = np.mod(
            self.base_state['position'][2] + np.pi, 2*np.pi) - np.pi
    
    def emergency_stop(self):
        """紧急停止"""
        self.emergency_stop_active = True
        self.base_state['target_velocity'].fill(0.0)
        self.base_state['velocity'].fill(0.0)
        self.base_state['joystick_input'].fill(0.0)
        logger.warning('Base emergency stop activated')
    
    def get_current_action(self):
        """获取当前动作（用于策略接口）"""
        if not self.episode_active or self.emergency_stop_active:
            return None
        
        # 检查超时
        if self.check_timeout():
            return None
        
        # 返回底盘控制动作
        return {
            'base_pose': self.base_state['position'].copy(),
            'base_velocity': self.base_state['velocity'].copy(),
            'target_velocity': self.base_state['target_velocity'].copy()
        }
    
    def get_status(self):
        """获取底盘状态（用于实时显示）"""
        if not self.episode_active:
            return None
        
        return {
            'position': {
                'x': float(self.base_state['position'][0]),
                'y': float(self.base_state['position'][1]),
                'theta': float(self.base_state['position'][2])
            },
            'velocity': {
                'vx': float(self.base_state['velocity'][0]),
                'vy': float(self.base_state['velocity'][1]),
                'wz': float(self.base_state['velocity'][2])
            },
            'target_velocity': {
                'vx': float(self.base_state['target_velocity'][0]),
                'vy': float(self.base_state['target_velocity'][1]),
                'wz': float(self.base_state['target_velocity'][2])
            },
            'joystick_input': {
                'x': float(self.base_state['joystick_input'][0]),
                'y': float(self.base_state['joystick_input'][1]),
                'z': float(self.base_state['joystick_input'][2])
            },
            'speed_scales': {
                'linear': self.linear_speed_scale,
                'angular': self.angular_speed_scale
            },
            'emergency_stop': self.emergency_stop_active,
            'timestamp': time.time()
        }
    
    def get_detailed_status(self):
        """获取详细状态信息"""
        return {
            'episode_active': self.episode_active,
            'power_save_mode': self.power_save_mode,
            'emergency_stop': self.emergency_stop_active,
            'max_speeds': {
                'linear': self.max_linear_speed,
                'angular': self.max_angular_speed
            },
            'current_speeds': {
                'linear': float(np.linalg.norm(self.base_state['velocity'][:2])),
                'angular': float(abs(self.base_state['velocity'][2]))
            },
            'speed_scales': {
                'linear': self.linear_speed_scale,
                'angular': self.angular_speed_scale
            },
            'last_control_time': self.last_control_time,
            'cache_size': len(self.control_cache)
        }
    
    def check_timeout(self):
        """检查控制超时"""
        if self.episode_active and (time.time() - self.last_control_time) > self.timeout_threshold:
            logger.warning('Base control timeout, emergency stopping')
            self.emergency_stop()
            return True
        return False
    
    def start_episode(self):
        """开始操作"""
        self.episode_active = True
        self.emergency_stop_active = False
        self.last_control_time = time.time()
        
        if self.power_save_mode:
            self.exit_power_save_mode()
        
        # 重置状态
        self.base_state['position'].fill(0.0)
        self.base_state['velocity'].fill(0.0)
        self.base_state['target_velocity'].fill(0.0)
        self.base_state['joystick_input'].fill(0.0)
        
        logger.info('Base episode started')
    
    def end_episode(self):
        """结束操作"""
        self.episode_active = False
        self.emergency_stop()
        logger.info('Base episode ended')
    
    def reset_environment(self):
        """重置环境"""
        self.episode_active = False
        self.emergency_stop_active = False
        
        # 重置所有状态
        self.base_state['position'].fill(0.0)
        self.base_state['velocity'].fill(0.0)
        self.base_state['target_velocity'].fill(0.0)
        self.base_state['joystick_input'].fill(0.0)
        
        # 清理缓存
        self.control_cache.clear()
        
        logger.info('Base environment reset')

class AdaptiveFilter:
    """自适应滤波器"""
    
    def __init__(self, alpha=0.7, adaptation_rate=0.1):
        self.alpha = alpha
        self.adaptation_rate = adaptation_rate
        self.last_value = None
        self.variance_estimate = 0.0
    
    def update(self, value):
        """更新滤波值，自适应调整参数"""
        if self.last_value is None:
            self.last_value = value.copy()
            return value
        
        # 计算变化量
        change = np.linalg.norm(value - self.last_value)
        
        # 自适应调整alpha
        if change > 0.1:  # 快速变化时
            effective_alpha = min(0.9, self.alpha + self.adaptation_rate)
        else:  # 缓慢变化时
            effective_alpha = max(0.3, self.alpha - self.adaptation_rate)
        
        # 应用滤波
        filtered = effective_alpha * value + (1 - effective_alpha) * self.last_value
        self.last_value = filtered.copy()
        
        return filtered

class PerformanceMonitor:
    """性能监控器"""
    
    def __init__(self):
        self.start_time = time.time()
        self.message_count = 0
        self.frame_count = 0
        self.last_stats_time = time.time()
        
        # 性能历史
        self.cpu_history = deque(maxlen=60)  # 1分钟历史
        self.memory_history = deque(maxlen=60)
        self.fps_history = deque(maxlen=60)
    
    def get_stats(self):
        """获取性能统计"""
        current_time = time.time()
        uptime = current_time - self.start_time
        
        # CPU和内存使用率
        cpu_percent = psutil.cpu_percent()
        memory_percent = psutil.virtual_memory().percent
        
        # 更新历史
        self.cpu_history.append(cpu_percent)
        self.memory_history.append(memory_percent)
        
        # 计算FPS
        time_delta = current_time - self.last_stats_time
        fps = self.frame_count / time_delta if time_delta > 0 else 0
        self.fps_history.append(fps)
        
        # 重置计数器
        self.frame_count = 0
        self.last_stats_time = current_time
        
        return {
            'uptime': uptime,
            'cpu_percent': cpu_percent,
            'memory_percent': memory_percent,
            'fps': fps,
            'avg_cpu': sum(self.cpu_history) / len(self.cpu_history) if self.cpu_history else 0,
            'avg_memory': sum(self.memory_history) / len(self.memory_history) if self.memory_history else 0,
            'avg_fps': sum(self.fps_history) / len(self.fps_history) if self.fps_history else 0,
            'message_count': self.message_count
        }
    
    def record_frame(self):
        """记录帧数"""
        self.frame_count += 1
    
    def record_message(self):
        """记录消息数"""
        self.message_count += 1

class BaseOptimizedAndroidTeleopPolicy(Policy):
    """优化的底盘Android遥控策略"""
    
    def __init__(self, host='0.0.0.0', port=5000):
        self.server = BaseOptimizedAndroidTeleopServer(host=host, port=port)
        self.server_thread = None
        
    def reset(self):
        """重置策略"""
        logger.info('Base Optimized Android Teleop Policy reset')
        
        if self.server_thread is None or not self.server_thread.is_alive():
            self.server_thread = threading.Thread(
                target=self.server.run, daemon=True
            )
            self.server_thread.start()
            
            time.sleep(2)
            logger.info(f'Base Optimized Android Teleop Server started')
            print(f'\n🚀 底盘优化版Android遥控服务器已启动!')
            print(f'📱 手机访问: http://{self._get_local_ip()}:{self.server.port}')
            print(f'🔧 调试界面: http://{self._get_local_ip()}:{self.server.port}/debug')
            print(f'📊 状态监控: http://{self._get_local_ip()}:{self.server.port}/status')
            print('=' * 60)
            print('🕹️ 底盘控制说明:')
            print('  • 左摇杆: 前后左右移动控制')
            print('  • 右按钮: 左半圆逆时针，右半圆顺时针旋转') 
            print('  • 速度滑动条: 调节移动和旋转速度')
            print('  • 手指离开: 立即停止运动')
            print('  • 红色按钮: 紧急停止')
            print('=' * 60)
    
    def _get_local_ip(self):
        """获取本地IP地址"""
        import socket
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))
            ip = s.getsockname()[0]
            s.close()
            return ip
        except:
            return '127.0.0.1'
    
    def step(self, obs):
        """执行一步策略"""
        # 检查超时
        self.server.controller.check_timeout()
        
        # 获取当前动作
        action = self.server.controller.get_current_action()
        
        # 如果有有效动作，返回底盘控制动作
        if action is not None:
            # 转换为标准的底盘控制格式
            return {
                'base_pose': action['base_pose'],
                'base_velocity': action['base_velocity']
            }
        
        return None

def main():
    """主函数"""
    import argparse
    from mujoco_env import MujocoEnv
    
    parser = argparse.ArgumentParser(description='TidyBot2 底盘优化Android遥控服务器')
    parser.add_argument('--host', default='0.0.0.0', help='服务器主机地址')
    parser.add_argument('--port', type=int, default=5000, help='服务器端口')
    parser.add_argument('--no-images', action='store_true', help='不显示仿真图像')
    parser.add_argument('--debug', action='store_true', help='启用调试模式')
    parser.add_argument('--max-linear-speed', type=float, default=0.5, help='最大线速度 (m/s)')
    parser.add_argument('--max-angular-speed', type=float, default=1.57, help='最大角速度 (rad/s)')
    args = parser.parse_args()
    
    print("📱 TidyBot2 底盘优化Android遥控服务器")
    print("=" * 50)
    print(f"🔧 配置参数:")
    print(f"   服务器地址: {args.host}:{args.port}")
    print(f"   最大线速度: {args.max_linear_speed} m/s")
    print(f"   最大角速度: {args.max_angular_speed} rad/s")
    print(f"   调试模式: {'启用' if args.debug else '禁用'}")
    print("=" * 50)
    
    env = None
    policy = None
    
    try:
        # 初始化仿真环境
        env = MujocoEnv(show_images=not args.no_images)
        
        # 初始化底盘遥控策略
        policy = BaseOptimizedAndroidTeleopPolicy(host=args.host, port=args.port)
        
        # 设置底盘参数
        if hasattr(policy.server.controller, 'max_linear_speed'):
            policy.server.controller.max_linear_speed = args.max_linear_speed
        if hasattr(policy.server.controller, 'max_angular_speed'):
            policy.server.controller.max_angular_speed = args.max_angular_speed
        
        print("🚀 正在启动系统...")
        
        # 主控制循环
        episode_count = 0
        while True:
            try:
                episode_count += 1
                print(f"\n📋 开始第 {episode_count} 次操作会话")
                
                # 重置环境和策略
                env.reset()
                policy.reset()
                
                # 等待用户开始操作
                print("⏳ 等待用户在手机上点击'开始操作'按钮...")
                
                step_count = 0
                last_status_time = time.time()
                
                while True:
                    try:
                        # 获取观测
                        obs = env.get_obs()
                        
                        # 执行策略
                        action = policy.step(obs)
                        
                        if action is None:
                            # 没有动作，等待
                            time.sleep(POLICY_CONTROL_PERIOD)
                            continue
                        elif isinstance(action, dict):
                            # 执行底盘动作
                            env.step(action)
                            step_count += 1
                            
                            # 定期打印状态
                            current_time = time.time()
                            if current_time - last_status_time > 10.0:  # 每10秒
                                controller_status = policy.server.controller.get_detailed_status()
                                if controller_status['episode_active']:
                                    print(f"📊 操作状态: 步数={step_count}, "
                                          f"线速度={controller_status['current_speeds']['linear']:.2f}m/s, "
                                          f"角速度={controller_status['current_speeds']['angular']:.2f}rad/s")
                                last_status_time = current_time
                        
                        # 检查紧急停止
                        if policy.server.controller.emergency_stop_active:
                            print("⚠️  检测到紧急停止，结束当前会话")
                            break
                        
                        # 检查操作是否结束
                        if not policy.server.controller.episode_active:
                            print(f"✅ 操作会话结束，总步数: {step_count}")
                            break
                        
                        time.sleep(POLICY_CONTROL_PERIOD)
                        
                    except KeyboardInterrupt:
                        print("\n⚠️  用户中断操作")
                        raise
                    except Exception as e:
                        logger.error(f"Control loop error: {e}")
                        print(f"❌ 控制循环错误: {e}")
                        # 尝试恢复
                        policy.server.controller.emergency_stop()
                        time.sleep(1)
                        break
                
            except KeyboardInterrupt:
                print("\n⚠️  用户中断程序")
                break
            except Exception as e:
                logger.error(f"Episode error: {e}")
                print(f"❌ 会话错误: {e}")
                print("🔄 尝试重新开始...")
                time.sleep(2)
                continue
    
    except KeyboardInterrupt:
        print("\n⚠️  程序被用户中断")
    except ImportError as e:
        print(f"❌ 导入错误: {e}")
        print("💡 请确保安装了所有必要的依赖包")
    except Exception as e:
        logger.error(f"Main error: {e}")
        print(f"❌ 发生严重错误: {e}")
    finally:
        print("\n🔧 正在清理资源...")
        
        # 清理策略
        if policy and policy.server:
            try:
                policy.server.controller.emergency_stop()
                policy.server.controller.reset_environment()
            except:
                pass
        
        # 清理环境
        if env:
            try:
                env.close()
            except:
                pass
        
        print("✅ 程序已安全退出")

if __name__ == '__main__':
    main() 