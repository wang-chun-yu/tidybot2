#!/usr/bin/env python3
"""
TidyBot2 传感器诊断工具

帮助排查Android设备传感器无法读取的问题
提供详细的诊断信息和解决方案

Author: AI Assistant
Date: September 2024
"""

import time
import json
from flask import Flask, render_template, jsonify, request
from flask_socketio import SocketIO, emit
import logging

# 配置日志
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class SensorDiagnosticServer:
    """传感器诊断服务器"""
    
    def __init__(self, host='0.0.0.0', port=5004):
        self.host = host
        self.port = port
        
        # Flask应用
        self.app = Flask(__name__)
        self.app.config['SECRET_KEY'] = 'sensor_diagnostic'
        
        # SocketIO
        self.socketio = SocketIO(
            self.app,
            cors_allowed_origins="*",
            logger=True,
            engineio_logger=True
        )
        
        # 诊断数据
        self.diagnostic_data = {}
        self.connected_clients = {}
        
        self.setup_routes()
        self.setup_socketio()
    
    def setup_routes(self):
        """设置路由"""
        
        @self.app.route('/')
        def index():
            return render_template('android_teleop_sensor_fixed.html')
        
        @self.app.route('/diagnostic')
        def diagnostic():
            """返回诊断报告"""
            return jsonify({
                'clients': len(self.connected_clients),
                'diagnostic_data': self.diagnostic_data,
                'timestamp': time.time()
            })
        
        @self.app.route('/solutions')
        def solutions():
            """返回解决方案建议"""
            solutions = {
                'android_chrome': {
                    'name': 'Android Chrome',
                    'common_issues': [
                        '需要用户手势触发权限请求',
                        '某些版本可能阻止传感器访问',
                        '需要HTTPS或localhost'
                    ],
                    'solutions': [
                        '确保点击"请求传感器权限"按钮',
                        '检查浏览器设置中的传感器权限',
                        '尝试使用最新版本Chrome',
                        '在地址栏中输入chrome://settings/content/sensors检查权限'
                    ]
                },
                'android_firefox': {
                    'name': 'Android Firefox',
                    'common_issues': [
                        '传感器API支持有限',
                        '需要在about:config中启用'
                    ],
                    'solutions': [
                        '在地址栏输入about:config',
                        '搜索device.sensors.enabled，设为true',
                        '重启浏览器后重试'
                    ]
                },
                'ios_safari': {
                    'name': 'iOS Safari',
                    'common_issues': [
                        'iOS 13+需要明确权限请求',
                        '需要HTTPS连接',
                        '需要用户手势触发'
                    ],
                    'solutions': [
                        '确保使用HTTPS访问',
                        '点击权限请求按钮',
                        '在设置->Safari->隐私与安全性中检查传感器权限'
                    ]
                },
                'general': {
                    'name': '通用解决方案',
                    'solutions': [
                        '尝试重新加载页面',
                        '清除浏览器缓存和Cookie',
                        '重启浏览器应用',
                        '检查设备的传感器是否正常工作（其他应用中测试）',
                        '确保设备未开启省电模式',
                        '尝试不同的浏览器'
                    ]
                }
            }
            return jsonify(solutions)
    
    def setup_socketio(self):
        """设置SocketIO事件"""
        
        @self.socketio.on('connect')
        def handle_connect():
            client_id = request.sid
            client_info = {
                'id': client_id,
                'connect_time': time.time(),
                'user_agent': request.headers.get('User-Agent', ''),
                'ip': request.remote_addr
            }
            
            self.connected_clients[client_id] = client_info
            logger.info(f'Diagnostic client connected: {client_id}')
            
            emit('connection_ack', {
                'client_id': client_id,
                'server_time': time.time()
            })
        
        @self.socketio.on('disconnect')
        def handle_disconnect():
            client_id = request.sid
            if client_id in self.connected_clients:
                del self.connected_clients[client_id]
            logger.info(f'Diagnostic client disconnected: {client_id}')
        
        @self.socketio.on('message')
        def handle_message(data):
            """处理诊断消息"""
            client_id = request.sid
            
            try:
                # 记录诊断数据
                if client_id not in self.diagnostic_data:
                    self.diagnostic_data[client_id] = {
                        'messages': [],
                        'sensor_data': {},
                        'last_update': time.time()
                    }
                
                # 添加时间戳
                data['server_timestamp'] = time.time()
                self.diagnostic_data[client_id]['messages'].append(data)
                self.diagnostic_data[client_id]['last_update'] = time.time()
                
                # 提取传感器数据
                if 'sensor' in data:
                    self.diagnostic_data[client_id]['sensor_data'] = data['sensor']
                
                if 'acceleration' in data:
                    self.diagnostic_data[client_id]['acceleration_data'] = data['acceleration']
                
                # 心跳回应
                if 'timestamp' in data:
                    emit('echo', data['timestamp'])
                
                # 分析数据并提供建议
                suggestions = self.analyze_sensor_data(client_id, data)
                if suggestions:
                    emit('diagnostic_suggestions', suggestions)
                
                logger.info(f'Diagnostic data from {client_id}: {type(data)} - {len(str(data))} chars')
                
            except Exception as e:
                logger.error(f'Error processing diagnostic message: {e}')
                emit('error', {'message': str(e)})
        
        @self.socketio.on('diagnostic_report')
        def handle_diagnostic_report(data):
            """处理诊断报告"""
            client_id = request.sid
            logger.info(f'Diagnostic report from {client_id}: {data}')
            
            # 存储诊断报告
            if client_id not in self.diagnostic_data:
                self.diagnostic_data[client_id] = {}
            
            self.diagnostic_data[client_id]['diagnostic_report'] = data
            self.diagnostic_data[client_id]['report_time'] = time.time()
            
            # 生成建议
            suggestions = self.generate_suggestions(data)
            emit('diagnostic_suggestions', suggestions)
    
    def analyze_sensor_data(self, client_id, data):
        """分析传感器数据并提供建议"""
        suggestions = []
        
        # 检查传感器数据
        if 'sensor' in data:
            sensor_data = data['sensor']
            
            # 检查数据是否全为零
            if (sensor_data.get('alpha', 0) == 0 and 
                sensor_data.get('beta', 0) == 0 and 
                sensor_data.get('gamma', 0) == 0):
                suggestions.append({
                    'type': 'warning',
                    'message': '传感器数据全为零，可能是权限未授予或设备静止'
                })
            else:
                suggestions.append({
                    'type': 'success',
                    'message': '传感器数据正常'
                })
        
        # 检查加速度数据
        if 'acceleration' in data:
            accel_data = data['acceleration']
            
            if (accel_data.get('x', 0) == 0 and 
                accel_data.get('y', 0) == 0 and 
                accel_data.get('z', 0) == 0):
                suggestions.append({
                    'type': 'warning',
                    'message': '加速度传感器数据为零'
                })
        
        return suggestions
    
    def generate_suggestions(self, diagnostic_data):
        """根据诊断数据生成建议"""
        suggestions = []
        
        browser = diagnostic_data.get('browser', 'unknown').lower()
        https_status = diagnostic_data.get('https_status', 'unknown')
        
        if 'chrome' in browser:
            if 'https' not in https_status:
                suggestions.append({
                    'type': 'error',
                    'message': 'Chrome需要HTTPS连接才能访问传感器',
                    'action': '请使用HTTPS访问或在localhost上测试'
                })
        
        if diagnostic_data.get('orientation_supported', False):
            if not diagnostic_data.get('permission_granted', False):
                suggestions.append({
                    'type': 'warning',
                    'message': '传感器支持但权限未授予',
                    'action': '请点击"请求传感器权限"按钮'
                })
        
        return suggestions
    
    def run(self):
        """启动诊断服务器"""
        logger.info(f'Starting Sensor Diagnostic Server on {self.host}:{self.port}')
        print(f'🔍 传感器诊断服务器启动')
        print(f'📱 访问地址: http://localhost:{self.port}')
        print(f'📊 诊断报告: http://localhost:{self.port}/diagnostic')
        print(f'💡 解决方案: http://localhost:{self.port}/solutions')
        print('=' * 60)
        
        self.socketio.run(
            self.app,
            host=self.host,
            port=self.port,
            debug=False,
            allow_unsafe_werkzeug=True
        )

def main():
    """主函数"""
    import argparse
    
    parser = argparse.ArgumentParser(description='TidyBot2 传感器诊断工具')
    parser.add_argument('--host', default='0.0.0.0', help='服务器主机地址')
    parser.add_argument('--port', type=int, default=5004, help='服务器端口')
    args = parser.parse_args()
    
    try:
        server = SensorDiagnosticServer(host=args.host, port=args.port)
        server.run()
    except KeyboardInterrupt:
        print('\n⏹️ 诊断服务器已停止')
    except Exception as e:
        print(f'❌ 服务器错误: {e}')

if __name__ == '__main__':
    main() 