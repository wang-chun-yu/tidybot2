#!/usr/bin/env python3
"""
TidyBot2 HTTPS Android遥控服务器

专门解决Chrome浏览器传感器权限问题
使用HTTPS协议启用传感器API

Author: AI Assistant
Date: September 2024
"""

import ssl
import time
from flask import Flask, render_template
from flask_socketio import SocketIO, emit
import logging

# 配置日志
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class HTTPSTeleopServer:
    """HTTPS Android遥控服务器"""
    
    def __init__(self, host='0.0.0.0', port=5443):
        self.host = host
        self.port = port
        
        # Flask应用
        self.app = Flask(__name__)
        self.app.config['SECRET_KEY'] = 'https_teleop_secret'
        
        # SocketIO配置
        self.socketio = SocketIO(
            self.app,
            cors_allowed_origins="*",
            logger=False,
            engineio_logger=False
        )
        
        # 客户端管理
        self.connected_clients = {}
        
        self.setup_routes()
        self.setup_socketio()
    
    def setup_routes(self):
        """设置路由"""
        
        @self.app.route('/')
        def index():
            return render_template('android_compatible_fix.html')
        
        @self.app.route('/sensor-test')
        def sensor_test():
            return render_template('android_teleop_sensor_fixed.html')
        
        @self.app.route('/debug')
        def debug():
            return render_template('android_debug_simple.html')
        
        @self.app.route('/health')
        def health():
            return {
                'status': 'ok',
                'protocol': 'https',
                'clients': len(self.connected_clients),
                'timestamp': time.time()
            }
    
    def setup_socketio(self):
        """设置SocketIO事件"""
        
        @self.socketio.on('connect')
        def handle_connect():
            from flask import request
            client_id = request.sid
            
            client_info = {
                'id': client_id,
                'connect_time': time.time(),
                'user_agent': request.headers.get('User-Agent', ''),
                'ip': request.remote_addr
            }
            
            self.connected_clients[client_id] = client_info
            logger.info(f'HTTPS Client connected: {client_id} from {request.remote_addr}')
            
            emit('connection_ack', {
                'client_id': client_id,
                'server_time': time.time(),
                'protocol': 'https'
            })
        
        @self.socketio.on('disconnect')
        def handle_disconnect():
            from flask import request
            client_id = request.sid
            
            if client_id in self.connected_clients:
                del self.connected_clients[client_id]
            
            logger.info(f'HTTPS Client disconnected: {client_id}')
        
        @self.socketio.on('message')
        def handle_message(data):
            """处理消息"""
            from flask import request
            client_id = request.sid
            
            try:
                # 添加服务器时间戳
                data['server_timestamp'] = time.time()
                
                # 心跳回应
                if 'timestamp' in data:
                    emit('echo', data['timestamp'])
                
                # 记录传感器数据
                if 'sensor' in data:
                    sensor_data = data['sensor']
                    logger.info(f'Sensor data from {client_id}: Alpha={sensor_data.get("alpha", 0):.1f}°')
                
                # 记录摇杆数据
                if 'joystick' in data:
                    joystick_data = data['joystick']
                    logger.info(f'Joystick from {client_id}: X={joystick_data.get("x", 0):.2f}, Y={joystick_data.get("y", 0):.2f}')
                
            except Exception as e:
                logger.error(f'Error processing message from {client_id}: {e}')
                emit('error', {'message': str(e)})
    
    def run(self):
        """启动HTTPS服务器"""
        logger.info(f'Starting HTTPS Android Teleop Server on {self.host}:{self.port}')
        
        # SSL上下文
        context = ssl.SSLContext(ssl.PROTOCOL_TLSv1_2)
        context.load_cert_chain('cert.pem', 'key.pem')
        
        print('🔒 HTTPS Android遥控服务器启动')
        print('=' * 60)
        print(f'🌐 HTTPS访问地址: https://129.68.11.102:{self.port}')
        print(f'🔧 兼容性修复: https://129.68.11.102:{self.port}/')
        print(f'🧪 传感器测试: https://129.68.11.102:{self.port}/sensor-test')
        print(f'🔍 调试页面: https://129.68.11.102:{self.port}/debug')
        print('=' * 60)
        print('⚠️  浏览器会显示"不安全连接"警告，请点击"高级"→"继续访问"')
        print('✅ HTTPS协议将启用Chrome传感器权限！')
        print('=' * 60)
        
        try:
            self.socketio.run(
                self.app,
                host=self.host,
                port=self.port,
                ssl_context=context,
                debug=False,
                allow_unsafe_werkzeug=True
            )
        except Exception as e:
            logger.error(f'HTTPS server error: {e}')
            print(f'❌ HTTPS服务器启动失败: {e}')

def main():
    """主函数"""
    import argparse
    
    parser = argparse.ArgumentParser(description='TidyBot2 HTTPS Android遥控服务器')
    parser.add_argument('--host', default='0.0.0.0', help='服务器主机地址')
    parser.add_argument('--port', type=int, default=5443, help='HTTPS服务器端口')
    args = parser.parse_args()
    
    # 检查证书文件
    import os
    if not os.path.exists('cert.pem') or not os.path.exists('key.pem'):
        print('❌ 缺少SSL证书文件！')
        print('请运行以下命令生成证书：')
        print('openssl req -x509 -newkey rsa:4096 -nodes -out cert.pem -keyout key.pem -days 365 -subj "/C=CN/ST=State/L=City/O=TidyBot2/CN=localhost"')
        return
    
    try:
        server = HTTPSTeleopServer(host=args.host, port=args.port)
        server.run()
    except KeyboardInterrupt:
        print('\n⏹️  HTTPS服务器已停止')
    except Exception as e:
        print(f'❌ 服务器错误: {e}')

if __name__ == '__main__':
    main() 