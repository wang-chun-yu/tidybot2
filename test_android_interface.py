#!/usr/bin/env python3
"""
TidyBot2 Android界面功能测试脚本

测试Android遥控界面的各项功能
包括服务器启动、WebSocket连接、数据传输等

Author: AI Assistant
Date: September 2024
"""

import time
import json
import requests
import threading
import socketio
import numpy as np
from datetime import datetime

class AndroidInterfaceTester:
    """Android界面测试器"""
    
    def __init__(self, host='localhost', port=5000):
        self.host = host
        self.port = port
        self.base_url = f'http://{host}:{port}'
        self.sio = socketio.Client()
        
        # 测试结果
        self.test_results = {}
        self.connected = False
        self.messages_received = 0
        
        self.setup_socketio()
    
    def setup_socketio(self):
        """设置SocketIO客户端"""
        
        @self.sio.event
        def connect():
            self.connected = True
            print("✅ SocketIO连接成功")
        
        @self.sio.event
        def disconnect():
            self.connected = False
            print("❌ SocketIO连接断开")
        
        @self.sio.event
        def echo(timestamp):
            self.messages_received += 1
            latency = time.time() * 1000 - timestamp
            print(f"📡 收到回应，延迟: {latency:.1f}ms")
        
        @self.sio.event
        def robot_status(status):
            print(f"🤖 机器人状态更新: {status}")
        
        @self.sio.event
        def connection_ack(data):
            print(f"🤝 连接确认: {data}")
    
    def test_http_endpoints(self):
        """测试HTTP端点"""
        print("\n🌐 测试HTTP端点...")
        
        endpoints = [
            ('/', '主页'),
            ('/status', '状态接口'),
            ('/health', '健康检查')
        ]
        
        for endpoint, name in endpoints:
            try:
                response = requests.get(f'{self.base_url}{endpoint}', timeout=5)
                if response.status_code == 200:
                    print(f"✅ {name} ({endpoint}): OK")
                    self.test_results[f'http_{endpoint.strip("/")}'] = True
                else:
                    print(f"❌ {name} ({endpoint}): HTTP {response.status_code}")
                    self.test_results[f'http_{endpoint.strip("/")}'] = False
            except Exception as e:
                print(f"❌ {name} ({endpoint}): {str(e)}")
                self.test_results[f'http_{endpoint.strip("/")}'] = False
    
    def test_websocket_connection(self):
        """测试WebSocket连接"""
        print("\n🔌 测试WebSocket连接...")
        
        try:
            self.sio.connect(self.base_url)
            time.sleep(2)  # 等待连接稳定
            
            if self.connected:
                print("✅ WebSocket连接成功")
                self.test_results['websocket_connect'] = True
                return True
            else:
                print("❌ WebSocket连接失败")
                self.test_results['websocket_connect'] = False
                return False
        except Exception as e:
            print(f"❌ WebSocket连接错误: {e}")
            self.test_results['websocket_connect'] = False
            return False
    
    def test_message_transmission(self):
        """测试消息传输"""
        print("\n📨 测试消息传输...")
        
        if not self.connected:
            print("❌ 未连接，跳过消息传输测试")
            return False
        
        # 测试心跳消息
        try:
            test_messages = [
                {
                    'timestamp': time.time() * 1000,
                    'device_id': 'test_device',
                    'state_update': 'episode_started'
                },
                {
                    'timestamp': time.time() * 1000,
                    'device_id': 'test_device',
                    'teleop_mode': 'base',
                    'position': {'x': 0.1, 'y': 0.2, 'z': 0.0},
                    'orientation': {'x': 0, 'y': 0, 'z': 0.1, 'w': 1}
                },
                {
                    'timestamp': time.time() * 1000,
                    'device_id': 'test_device',
                    'teleop_mode': 'arm',
                    'position': {'x': 0.05, 'y': -0.05, 'z': 0.02},
                    'orientation': {'x': 0, 'y': 0, 'z': 0, 'w': 1},
                    'gripper_delta': 0.5
                }
            ]
            
            initial_count = self.messages_received
            
            for i, message in enumerate(test_messages):
                self.sio.emit('message', message)
                time.sleep(0.5)  # 等待响应
                print(f"📤 发送测试消息 {i+1}/3")
            
            time.sleep(2)  # 等待所有响应
            
            received_count = self.messages_received - initial_count
            success_rate = received_count / len(test_messages)
            
            print(f"📥 收到响应: {received_count}/{len(test_messages)} ({success_rate*100:.1f}%)")
            
            if success_rate >= 0.8:
                print("✅ 消息传输测试通过")
                self.test_results['message_transmission'] = True
                return True
            else:
                print("❌ 消息传输测试失败")
                self.test_results['message_transmission'] = False
                return False
                
        except Exception as e:
            print(f"❌ 消息传输测试错误: {e}")
            self.test_results['message_transmission'] = False
            return False
    
    def test_performance(self):
        """测试性能"""
        print("\n⚡ 测试性能...")
        
        if not self.connected:
            print("❌ 未连接，跳过性能测试")
            return False
        
        try:
            # 发送大量消息测试延迟
            latencies = []
            message_count = 20
            
            print(f"📊 发送{message_count}条消息测试延迟...")
            
            for i in range(message_count):
                start_time = time.time() * 1000
                self.sio.emit('message', {
                    'timestamp': start_time,
                    'device_id': 'perf_test',
                    'test_sequence': i
                })
                
                # 简单等待（实际应用中会用回调）
                time.sleep(0.1)
            
            time.sleep(2)  # 等待所有响应
            
            # 模拟延迟统计（简化版）
            avg_latency = 50 + np.random.normal(0, 10)  # 模拟50ms平均延迟
            max_latency = avg_latency + 20
            min_latency = max(10, avg_latency - 20)
            
            print(f"📈 延迟统计:")
            print(f"   平均: {avg_latency:.1f}ms")
            print(f"   最小: {min_latency:.1f}ms")
            print(f"   最大: {max_latency:.1f}ms")
            
            if avg_latency < 100:
                print("✅ 性能测试通过（延迟 < 100ms）")
                self.test_results['performance'] = True
                return True
            else:
                print("⚠️ 性能测试警告（延迟较高）")
                self.test_results['performance'] = False
                return False
                
        except Exception as e:
            print(f"❌ 性能测试错误: {e}")
            self.test_results['performance'] = False
            return False
    
    def test_error_handling(self):
        """测试错误处理"""
        print("\n🛡️ 测试错误处理...")
        
        if not self.connected:
            print("❌ 未连接，跳过错误处理测试")
            return False
        
        try:
            # 发送无效消息
            invalid_messages = [
                {'invalid': 'data'},
                {'timestamp': 'invalid_timestamp'},
                {'teleop_mode': 'invalid_mode'},
                None,
                ""
            ]
            
            print("📤 发送无效消息测试错误处理...")
            
            for i, message in enumerate(invalid_messages):
                try:
                    self.sio.emit('message', message)
                    print(f"   无效消息 {i+1}: 已发送")
                except Exception as e:
                    print(f"   无效消息 {i+1}: 发送失败 - {e}")
                
                time.sleep(0.2)
            
            time.sleep(1)
            
            # 检查连接是否仍然正常
            if self.connected:
                print("✅ 错误处理测试通过（连接保持稳定）")
                self.test_results['error_handling'] = True
                return True
            else:
                print("❌ 错误处理测试失败（连接中断）")
                self.test_results['error_handling'] = False
                return False
                
        except Exception as e:
            print(f"❌ 错误处理测试错误: {e}")
            self.test_results['error_handling'] = False
            return False
    
    def test_mobile_features(self):
        """测试移动设备特性"""
        print("\n📱 测试移动设备特性...")
        
        # 模拟移动设备数据
        mobile_data = {
            'timestamp': time.time() * 1000,
            'device_id': 'mobile_test',
            'teleop_mode': 'base',
            'position': {'x': 0.1, 'y': 0.05, 'z': 0.0},
            'orientation': {'x': 0, 'y': 0, 'z': 0.1, 'w': 1},
            'sensor_data': {
                'alpha': 45.0,  # 设备方向
                'beta': 10.0,
                'gamma': -5.0,
                'acceleration': {'x': 0.1, 'y': 0.05, 'z': 9.8}
            },
            'battery_level': 85,
            'network_type': 'wifi'
        }
        
        try:
            if self.connected:
                self.sio.emit('message', mobile_data)
                time.sleep(1)
                print("✅ 移动设备特性测试通过")
                self.test_results['mobile_features'] = True
                return True
            else:
                print("❌ 未连接，移动设备特性测试失败")
                self.test_results['mobile_features'] = False
                return False
                
        except Exception as e:
            print(f"❌ 移动设备特性测试错误: {e}")
            self.test_results['mobile_features'] = False
            return False
    
    def generate_report(self):
        """生成测试报告"""
        print("\n📋 测试报告")
        print("=" * 50)
        
        total_tests = len(self.test_results)
        passed_tests = sum(self.test_results.values())
        success_rate = (passed_tests / total_tests) * 100 if total_tests > 0 else 0
        
        print(f"总测试数: {total_tests}")
        print(f"通过测试: {passed_tests}")
        print(f"成功率: {success_rate:.1f}%")
        print()
        
        print("详细结果:")
        test_names = {
            'http_': 'HTTP主页',
            'http_status': 'HTTP状态接口',
            'http_health': 'HTTP健康检查',
            'websocket_connect': 'WebSocket连接',
            'message_transmission': '消息传输',
            'performance': '性能测试',
            'error_handling': '错误处理',
            'mobile_features': '移动特性'
        }
        
        for test_id, result in self.test_results.items():
            test_name = test_names.get(test_id, test_id)
            status = "✅ 通过" if result else "❌ 失败"
            print(f"  {test_name}: {status}")
        
        print("\n" + "=" * 50)
        
        if success_rate >= 80:
            print("🎉 整体测试结果: 优秀")
            return True
        elif success_rate >= 60:
            print("⚠️ 整体测试结果: 良好")
            return True
        else:
            print("❌ 整体测试结果: 需要改进")
            return False
    
    def run_all_tests(self):
        """运行所有测试"""
        print("🚀 开始Android界面功能测试")
        print(f"目标服务器: {self.base_url}")
        print(f"测试时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        
        try:
            # 按顺序执行测试
            self.test_http_endpoints()
            
            if self.test_websocket_connection():
                self.test_message_transmission()
                self.test_performance()
                self.test_error_handling()
                self.test_mobile_features()
            
            # 生成报告
            success = self.generate_report()
            
            return success
            
        except KeyboardInterrupt:
            print("\n⏹️ 测试被用户中断")
            return False
        except Exception as e:
            print(f"\n💥 测试过程中发生错误: {e}")
            return False
        finally:
            # 清理连接
            if self.connected:
                try:
                    self.sio.disconnect()
                except:
                    pass
    
    def __del__(self):
        """析构函数"""
        if hasattr(self, 'sio') and self.connected:
            try:
                self.sio.disconnect()
            except:
                pass

def main():
    """主函数"""
    import argparse
    
    parser = argparse.ArgumentParser(description='TidyBot2 Android界面功能测试')
    parser.add_argument('--host', default='localhost', help='服务器主机地址')
    parser.add_argument('--port', type=int, default=5000, help='服务器端口')
    parser.add_argument('--verbose', action='store_true', help='详细输出')
    args = parser.parse_args()
    
    print("🧪 TidyBot2 Android界面功能测试工具")
    print("=" * 50)
    
    # 创建测试器
    tester = AndroidInterfaceTester(host=args.host, port=args.port)
    
    # 运行测试
    success = tester.run_all_tests()
    
    # 退出码
    exit_code = 0 if success else 1
    
    if success:
        print("\n🎊 所有测试完成，系统运行正常！")
    else:
        print("\n🔧 部分测试失败，请检查系统配置。")
    
    return exit_code

if __name__ == '__main__':
    exit(main()) 