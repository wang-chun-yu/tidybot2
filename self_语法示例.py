#!/usr/bin/env python3
"""
Python self 语法详解示例

self 是 Python 面向对象编程的核心概念
"""

class Robot:
    """机器人类 - 用于演示self的作用"""
    
    # 类属性（所有实例共享）
    robot_count = 0
    
    def __init__(self, name, model):
        """
        构造函数 - self指向新创建的实例
        
        Args:
            self: 指向当前创建的Robot实例
            name: 机器人名称
            model: 机器人型号
        """
        # 通过self给实例添加属性
        self.name = name          # 实例属性：机器人名称
        self.model = model        # 实例属性：机器人型号
        self.is_running = False   # 实例属性：运行状态
        self.speed = 0.0         # 实例属性：当前速度
        
        # 修改类属性
        Robot.robot_count += 1
        
        print(f"创建机器人: {self.name} (型号: {self.model})")
    
    def start(self):
        """
        启动机器人
        
        Args:
            self: 指向调用此方法的Robot实例
        """
        if not self.is_running:  # 访问实例属性
            self.is_running = True   # 修改实例属性
            print(f"{self.name} 已启动")
        else:
            print(f"{self.name} 已经在运行中")
    
    def stop(self):
        """停止机器人"""
        if self.is_running:
            self.is_running = False
            self.speed = 0.0
            print(f"{self.name} 已停止")
        else:
            print(f"{self.name} 已经停止了")
    
    def set_speed(self, new_speed):
        """
        设置机器人速度
        
        Args:
            self: 当前实例
            new_speed: 新的速度值
        """
        if self.is_running:  # 检查实例状态
            self.speed = new_speed  # 修改实例属性
            print(f"{self.name} 速度设置为: {self.speed} m/s")
        else:
            print(f"{self.name} 未启动，无法设置速度")
    
    def get_status(self):
        """获取机器人状态 - 返回实例信息"""
        return {
            'name': self.name,
            'model': self.model,
            'is_running': self.is_running,
            'speed': self.speed
        }
    
    def move_to_position(self, x, y):
        """移动到指定位置"""
        if self.is_running:
            print(f"{self.name} 移动到位置 ({x}, {y})")
            # 调用其他实例方法
            self._update_position(x, y)
        else:
            print(f"{self.name} 未启动，无法移动")
    
    def _update_position(self, x, y):
        """私有方法 - 更新位置（通过self调用）"""
        self.current_x = x  # 添加新的实例属性
        self.current_y = y
        print(f"{self.name} 位置已更新为 ({x}, {y})")
    
    def __str__(self):
        """字符串表示 - self指向当前实例"""
        return f"Robot(name='{self.name}', model='{self.model}', running={self.is_running})"
    
    @classmethod
    def get_robot_count(cls):
        """类方法 - 使用cls而不是self"""
        return cls.robot_count
    
    @staticmethod
    def calculate_distance(x1, y1, x2, y2):
        """静态方法 - 不需要self或cls"""
        return ((x2-x1)**2 + (y2-y1)**2)**0.5

def demonstrate_self_usage():
    """演示self的使用"""
    
    print("=" * 50)
    print("Python self 语法演示")
    print("=" * 50)
    
    # 创建两个不同的机器人实例
    robot1 = Robot("TidyBot1", "清洁型")
    robot2 = Robot("TidyBot2", "搬运型")
    
    print(f"\n当前机器人总数: {Robot.get_robot_count()}")
    
    print("\n1. 每个实例都有独立的属性:")
    print(f"robot1.name = {robot1.name}")
    print(f"robot2.name = {robot2.name}")
    
    print("\n2. self让方法能够访问和修改实例属性:")
    robot1.start()  # self指向robot1
    robot2.start()  # self指向robot2
    
    print("\n3. 不同实例的状态是独立的:")
    robot1.set_speed(0.5)  # 只影响robot1的速度
    robot2.set_speed(1.0)  # 只影响robot2的速度
    
    print(f"robot1状态: {robot1.get_status()}")
    print(f"robot2状态: {robot2.get_status()}")
    
    print("\n4. self允许方法调用其他实例方法:")
    robot1.move_to_position(10, 20)  # 内部会调用_update_position
    
    print("\n5. 不同实例调用相同方法，self指向不同对象:")
    print(f"robot1: {robot1}")  # self指向robot1
    print(f"robot2: {robot2}")  # self指向robot2
    
    print("\n6. 验证属性确实属于各自实例:")
    print(f"robot1是否在运行: {robot1.is_running}")
    print(f"robot2是否在运行: {robot2.is_running}")
    
    # 停止一个，不影响另一个
    robot1.stop()
    print(f"停止robot1后:")
    print(f"robot1是否在运行: {robot1.is_running}")
    print(f"robot2是否在运行: {robot2.is_running}")

class VehicleExample:
    """模拟您的Vehicle类的简化版本"""
    
    def __init__(self, max_speed):
        # 这里的self指向新创建的Vehicle实例
        self.max_speed = max_speed
        self.current_speed = 0.0
        self.is_active = False
        
        # 初始化控制循环 - 调用其他方法
        self._init_control_system()
    
    def _init_control_system(self):
        """初始化控制系统 - 类似您代码中的方法"""
        # self指向调用此方法的Vehicle实例
        self.command_queue = []  # 简化的命令队列
        self.control_active = False
        print(f"Vehicle控制系统已初始化，最大速度: {self.max_speed}")
    
    def start_control(self):
        """启动控制"""
        self.is_active = True
        self.control_active = True
        print("Vehicle控制已启动")
    
    def set_speed(self, speed):
        """设置速度"""
        if speed <= self.max_speed:  # 使用self访问实例属性
            self.current_speed = speed
            print(f"速度设置为: {self.current_speed}")
        else:
            print(f"速度{speed}超过最大限制{self.max_speed}")

def demonstrate_vehicle_example():
    """演示Vehicle类的self使用"""
    print("\n" + "=" * 50)
    print("Vehicle类中self的使用演示")
    print("=" * 50)
    
    # 创建Vehicle实例
    vehicle = VehicleExample(max_speed=2.0)
    
    # 每次调用方法时，self都指向这个vehicle实例
    vehicle.start_control()
    vehicle.set_speed(1.5)
    
    print(f"当前Vehicle状态:")
    print(f"  最大速度: {vehicle.max_speed}")
    print(f"  当前速度: {vehicle.current_speed}")
    print(f"  是否激活: {vehicle.is_active}")

if __name__ == "__main__":
    # 运行演示
    demonstrate_self_usage()
    demonstrate_vehicle_example()
    
    print("\n" + "=" * 50)
    print("self语法要点总结:")
    print("=" * 50)
    print("1. self是方法的第一个参数，代表调用方法的实例")
    print("2. 通过self可以访问和修改实例属性")
    print("3. 通过self可以调用其他实例方法")
    print("4. 不同实例的self指向不同的对象")
    print("5. self让每个实例都有独立的状态和行为")
    print("6. 在方法定义时必须写self，调用时Python自动传入") 