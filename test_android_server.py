#!/usr/bin/env python3
"""
Android遥控服务器简化测试版本
用于验证修复是否有效

Author: AI Assistant
Date: September 2024
"""

import time
import logging
from flask import Flask, jsonify, request
from flask_socketio import SocketIO, emit

# 配置日志
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class SimpleAndroidTeleopServer:
    """简化的Android遥控服务器"""
    
    def __init__(self, host='0.0.0.0', port=5000):
        self.host = host
        self.port = port
        
        # Flask应用
        self.app = Flask(__name__)
        self.app.config['SECRET_KEY'] = 'test_key'
        
        # SocketIO配置（修复版）
        self.socketio = SocketIO(
            self.app, 
            cors_allowed_origins="*",
            logger=False,
            engineio_logger=False
        )
        
        # 简单状态
        self.connected_clients = set()
        self.message_count = 0
        
        self.setup_routes()
        self.setup_socketio()
    
    def setup_routes(self):
        """设置路由"""
        
        @self.app.route('/')
        def index():
            return jsonify({
                'status': 'running',
                'message': 'Android Teleop Server Test',
                'clients': len(self.connected_clients),
                'messages': self.message_count
            })
        
        @self.app.route('/health')
        def health():
            return jsonify({
                'status': 'healthy', 
                'timestamp': time.time()
            })
    
    def setup_socketio(self):
        """设置SocketIO事件"""
        
        @self.socketio.on('connect')
        def handle_connect():
            client_id = request.sid if 'request' in globals() else 'test_client'
            self.connected_clients.add(client_id)
            logger.info(f'Client connected: {client_id}')
            emit('connection_ack', {'status': 'connected'})
        
        @self.socketio.on('disconnect')
        def handle_disconnect():
            client_id = request.sid if 'request' in globals() else 'test_client'
            self.connected_clients.discard(client_id)
            logger.info(f'Client disconnected: {client_id}')
        
        @self.socketio.on('message')
        def handle_message(data):
            self.message_count += 1
            logger.info(f'Received message #{self.message_count}: {type(data)}')
            
            # 简单回应
            if isinstance(data, dict) and 'timestamp' in data:
                emit('echo', data['timestamp'])
    
    def run(self):
        """启动服务器（修复版）"""
        logger.info(f'Starting Simple Android Teleop Server on {self.host}:{self.port}')
        
        try:
            # 修复：移除冲突的参数
            self.socketio.run(
                self.app,
                host=self.host,
                port=self.port,
                debug=False,
                allow_unsafe_werkzeug=True,
                use_reloader=False
                # 不再传递 threaded=True
            )
        except Exception as e:
            logger.error(f'Server startup error: {e}')
            raise

class SimpleAndroidTeleopController:
    """简化的控制器"""
    
    def __init__(self):
        self.episode_active = False
        self.last_data_time = time.time()
        self.timeout_threshold = 2.0
        
        logger.info('Simple Android Teleop Controller initialized')
    
    def check_timeout(self):
        """检查超时（修复：添加缺失的方法）"""
        if self.episode_active and (time.time() - self.last_data_time) > self.timeout_threshold:
            logger.warning('Teleop data timeout, stopping episode')
            self.episode_active = False
            return True
        return False
    
    def get_current_action(self):
        """获取当前动作"""
        if not self.episode_active:
            return None
        
        return {
            'base_pose': [0.0, 0.0, 0.0],
            'arm_pos': [0.55, 0.0, 0.4],
            'arm_quat': [0.0, 0.0, 0.0, 1.0],
            'gripper_pos': [0.5]
        }

def test_server():
    """测试服务器功能"""
    print("🧪 Android遥控服务器修复测试")
    print("=" * 50)
    
    try:
        # 创建服务器
        server = SimpleAndroidTeleopServer(port=5003)
        controller = SimpleAndroidTeleopController()
        
        print("✅ 服务器对象创建成功")
        
        # 测试控制器方法
        timeout_result = controller.check_timeout()
        action = controller.get_current_action()
        
        print(f"✅ 控制器方法测试成功:")
        print(f"   check_timeout(): {timeout_result}")
        print(f"   get_current_action(): {action is not None}")
        
        print("\n🚀 启动服务器...")
        print(f"访问地址: http://localhost:5003")
        print("按 Ctrl+C 停止服务器")
        
        # 启动服务器
        server.run()
        
    except KeyboardInterrupt:
        print("\n⏹️ 服务器已停止")
    except Exception as e:
        print(f"\n❌ 错误: {e}")
        import traceback
        traceback.print_exc()

if __name__ == '__main__':
    test_server() 