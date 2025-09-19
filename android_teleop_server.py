#!/usr/bin/env python3
"""
TidyBot2 Android增强遥控服务器

支持增强版Android Web界面的服务器端实现
提供完整的遥控功能和状态反馈

Author: AI Assistant
Date: September 2024
"""

import time
import json
import numpy as np
import threading
from flask import Flask, render_template, request
from flask_socketio import SocketIO, emit
import logging

from constants import POLICY_CONTROL_PERIOD
from policies import Policy

# 配置日志
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class AndroidTeleopServer:
    """Android遥控服务器类"""
    
    def __init__(self, host='0.0.0.0', port=5000, debug=False):
        self.host = host
        self.port = port
        self.debug = debug
        
        # Flask应用设置
        self.app = Flask(__name__)
        self.app.config['SECRET_KEY'] = 'tidybot2_android_teleop'
        self.socketio = SocketIO(self.app, cors_allowed_origins="*", 
                                logger=debug, engineio_logger=debug)
        
        # 遥控状态
        self.controller = AndroidTeleopController()
        self.connected_clients = set()
        self.robot_status = {
            'mode': 'idle',
            'message': '准备就绪',
            'episode_active': False,
            'last_update': time.time()
        }
        
        self.setup_routes()
        self.setup_socketio()
    
    def setup_routes(self):
        """设置Flask路由"""
        
        @self.app.route('/')
        def index():
            return render_template('android_teleop_enhanced.html')
        
        @self.app.route('/status')
        def status():
            """返回服务器状态"""
            return {
                'status': 'running',
                'clients': len(self.connected_clients),
                'robot_status': self.robot_status,
                'timestamp': time.time()
            }
        
        @self.app.route('/health')
        def health():
            """健康检查端点"""
            return {'status': 'healthy', 'timestamp': time.time()}
    
    def setup_socketio(self):
        """设置SocketIO事件处理"""
        
        @self.socketio.on('connect')
        def handle_connect():
            client_id = request.sid
            self.connected_clients.add(client_id)
            logger.info(f'Client {client_id} connected. Total clients: {len(self.connected_clients)}')
            
            # 发送欢迎消息和当前状态
            emit('robot_status', self.robot_status)
            
            # 启动心跳
            self.start_heartbeat(client_id)
        
        @self.socketio.on('disconnect')
        def handle_disconnect():
            client_id = request.sid
            self.connected_clients.discard(client_id)
            logger.info(f'Client {client_id} disconnected. Total clients: {len(self.connected_clients)}')
            
            # 如果没有客户端连接，停止当前操作
            if len(self.connected_clients) == 0:
                self.controller.emergency_stop()
                self.update_robot_status('idle', '等待连接')
        
        @self.socketio.on('message')
        def handle_message(data):
            """处理客户端消息"""
            try:
                # 解析消息
                timestamp = data.get('timestamp', time.time() * 1000)
                device_id = data.get('device_id', 'unknown')
                
                # 发送心跳回应
                emit('echo', timestamp)
                
                # 处理状态更新
                if 'state_update' in data:
                    state = data['state_update']
                    if state == 'episode_started':
                        self.controller.start_episode()
                        self.update_robot_status('operating', '操作中')
                    elif state == 'episode_ended':
                        self.controller.end_episode()
                        self.update_robot_status('idle', '操作结束')
                    elif state == 'reset_env':
                        self.controller.reset_environment()
                        self.update_robot_status('resetting', '重置环境中')
                
                # 处理遥控数据
                elif 'teleop_mode' in data:
                    self.controller.process_teleop_data(data)
                
            except Exception as e:
                logger.error(f'Error handling message: {e}')
                emit('error', {'message': str(e)})
    
    def start_heartbeat(self, client_id):
        """为客户端启动心跳"""
        def heartbeat():
            while client_id in self.connected_clients:
                try:
                    self.socketio.emit('heartbeat', {'timestamp': time.time()}, 
                                     room=client_id)
                    time.sleep(5)  # 5秒心跳间隔
                except:
                    break
        
        thread = threading.Thread(target=heartbeat, daemon=True)
        thread.start()
    
    def update_robot_status(self, mode, message):
        """更新机器人状态"""
        self.robot_status.update({
            'mode': mode,
            'message': message,
            'last_update': time.time()
        })
        
        # 广播状态更新
        self.socketio.emit('robot_status', self.robot_status)
        logger.info(f'Robot status updated: {mode} - {message}')
    
    def run(self):
        """启动服务器"""
        logger.info(f'Starting Android Teleop Server on {self.host}:{self.port}')
        self.socketio.run(self.app, host=self.host, port=self.port, 
                         debug=self.debug, allow_unsafe_werkzeug=True)

class AndroidTeleopController:
    """Android遥控控制器"""
    
    def __init__(self):
        # 遥控状态
        self.episode_active = False
        self.current_mode = 'base'
        self.last_data_time = time.time()
        
        # 控制数据
        self.base_pose = np.array([0.0, 0.0, 0.0])
        self.arm_pos = np.array([0.55, 0.0, 0.4])
        self.arm_quat = np.array([0.0, 0.0, 0.0, 1.0])
        self.gripper_pos = np.array([0.5])
        
        # 数据平滑
        self.position_filter = ExponentialFilter(alpha=0.7)
        self.orientation_filter = ExponentialFilter(alpha=0.5)
        
        # 安全参数
        self.max_position_change = 0.1  # 最大位置变化
        self.max_orientation_change = 0.2  # 最大姿态变化
        self.timeout_threshold = 1.0  # 数据超时阈值
        
        logger.info('Android Teleop Controller initialized')
    
    def start_episode(self):
        """开始操作"""
        self.episode_active = True
        self.last_data_time = time.time()
        logger.info('Episode started')
    
    def end_episode(self):
        """结束操作"""
        self.episode_active = False
        logger.info('Episode ended')
    
    def reset_environment(self):
        """重置环境"""
        self.episode_active = False
        self.base_pose = np.array([0.0, 0.0, 0.0])
        self.arm_pos = np.array([0.55, 0.0, 0.4])
        self.arm_quat = np.array([0.0, 0.0, 0.0, 1.0])
        self.gripper_pos = np.array([0.5])
        logger.info('Environment reset')
    
    def emergency_stop(self):
        """紧急停止"""
        self.episode_active = False
        logger.warning('Emergency stop triggered')
    
    def process_teleop_data(self, data):
        """处理遥控数据"""
        if not self.episode_active:
            return
        
        try:
            # 更新数据时间
            self.last_data_time = time.time()
            
            # 获取控制模式
            mode = data.get('teleop_mode', 'base')
            self.current_mode = mode
            
            # 获取位置和姿态数据
            position = data.get('position', {})
            orientation = data.get('orientation', {})
            
            # 数据验证和限制
            pos_delta = np.array([
                self._clamp(position.get('x', 0), -self.max_position_change, self.max_position_change),
                self._clamp(position.get('y', 0), -self.max_position_change, self.max_position_change),
                self._clamp(position.get('z', 0), -self.max_position_change, self.max_position_change)
            ])
            
            # 应用数据平滑
            pos_delta = self.position_filter.update(pos_delta)
            
            if mode == 'base':
                # 底盘控制
                self.base_pose[:2] += pos_delta[:2]
                
                # 旋转控制
                rot_z = self._clamp(orientation.get('z', 0), 
                                   -self.max_orientation_change, 
                                   self.max_orientation_change)
                rot_z = self.orientation_filter.update(np.array([rot_z]))[0]
                self.base_pose[2] += rot_z
                
                # 限制底盘范围
                self.base_pose[:2] = np.clip(self.base_pose[:2], -2.0, 2.0)
                self.base_pose[2] = np.mod(self.base_pose[2] + np.pi, 2*np.pi) - np.pi
                
            elif mode == 'arm':
                # 机械臂控制
                self.arm_pos += pos_delta
                
                # 限制工作空间
                self.arm_pos[0] = np.clip(self.arm_pos[0], 0.2, 0.8)
                self.arm_pos[1] = np.clip(self.arm_pos[1], -0.4, 0.4)
                self.arm_pos[2] = np.clip(self.arm_pos[2], 0.1, 0.8)
                
                # 夹爪控制
                if 'gripper_delta' in data:
                    gripper_delta = self._clamp(data['gripper_delta'], -0.1, 0.1)
                    self.gripper_pos[0] += gripper_delta
                    self.gripper_pos[0] = np.clip(self.gripper_pos[0], 0.0, 1.0)
            
        except Exception as e:
            logger.error(f'Error processing teleop data: {e}')
    
    def _clamp(self, value, min_val, max_val):
        """限制数值范围"""
        return max(min_val, min(max_val, value))
    
    def check_timeout(self):
        """检查数据超时"""
        if self.episode_active and (time.time() - self.last_data_time) > self.timeout_threshold:
            logger.warning('Teleop data timeout, stopping episode')
            self.emergency_stop()
            return True
        return False
    
    def get_current_action(self):
        """获取当前动作"""
        if not self.episode_active:
            return None
        
        return {
            'base_pose': self.base_pose.copy(),
            'arm_pos': self.arm_pos.copy(),
            'arm_quat': self.arm_quat.copy(),
            'gripper_pos': self.gripper_pos.copy(),
        }

class ExponentialFilter:
    """指数平滑滤波器"""
    
    def __init__(self, alpha=0.7):
        self.alpha = alpha
        self.last_value = None
    
    def update(self, value):
        """更新滤波值"""
        if self.last_value is None:
            self.last_value = value
            return value
        
        filtered = self.alpha * value + (1 - self.alpha) * self.last_value
        self.last_value = filtered
        return filtered

class AndroidTeleopPolicy(Policy):
    """Android遥控策略"""
    
    def __init__(self, host='0.0.0.0', port=5000):
        self.server = AndroidTeleopServer(host=host, port=port)
        self.server_thread = None
        
    def reset(self):
        """重置策略"""
        logger.info('Android Teleop Policy reset')
        
        # 启动服务器（如果未启动）
        if self.server_thread is None or not self.server_thread.is_alive():
            self.server_thread = threading.Thread(
                target=self.server.run, daemon=True
            )
            self.server_thread.start()
            
            # 等待服务器启动
            time.sleep(2)
            logger.info(f'Android Teleop Server started at http://{self.server.host}:{self.server.port}')
            print(f'\n🚀 Android遥控服务器已启动!')
            print(f'📱 请在手机浏览器中访问: http://{self._get_local_ip()}:{self.server.port}')
            print(f'💻 或在本地访问: http://localhost:{self.server.port}')
            print('=' * 60)
    
    def _get_local_ip(self):
        """获取本地IP地址"""
        import socket
        try:
            # 连接到外部地址来获取本地IP
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))
            ip = s.getsockname()[0]
            s.close()
            return ip
        except:
            return '127.0.0.1'
    
    def step(self, obs):
        """执行一步策略"""
        # 检查数据超时
        self.server.controller.check_timeout()
        
        # 返回当前动作
        return self.server.controller.get_current_action()

def main():
    """主函数 - 独立测试Android遥控服务器"""
    import argparse
    from mujoco_env import MujocoEnv
    
    parser = argparse.ArgumentParser(description='TidyBot2 Android遥控服务器测试')
    parser.add_argument('--host', default='0.0.0.0', help='服务器主机地址')
    parser.add_argument('--port', type=int, default=5000, help='服务器端口')
    parser.add_argument('--no-images', action='store_true', help='不显示仿真图像')
    parser.add_argument('--debug', action='store_true', help='启用调试模式')
    args = parser.parse_args()
    
    print("📱 TidyBot2 Android遥控服务器测试")
    print("=" * 50)
    
    try:
        # 创建环境和策略
        env = MujocoEnv(show_images=not args.no_images)
        policy = AndroidTeleopPolicy(host=args.host, port=args.port)
        
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
        print("程序已退出")

if __name__ == '__main__':
    main() 