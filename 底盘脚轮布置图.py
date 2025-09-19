#!/usr/bin/env python3
"""
TidyBot2 底盘脚轮布置图绘制脚本

根据实际的物理参数绘制底盘脚轮的布置图，包括：
- 脚轮位置和编号
- 转向轴和接触点
- 尺寸标注
- 坐标系显示

Author: AI Assistant
Date: September 2024
"""

import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np

# 设置中文字体
plt.rcParams['font.sans-serif'] = ['SimHei', 'Arial Unicode MS', 'DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False

# 底盘物理参数（从constants.py和base_controller.py获取）
h_x = np.array([0.190150, 0.190150, -0.190150, -0.190150])  # 脚轮中心x坐标
h_y = np.array([-0.170150, 0.170150, 0.170150, -0.170150])  # 脚轮中心y坐标
b_x = -0.014008  # 转向轴到接触点的前后偏移
b_y = -0.003753  # 转向轴到接触点的左右偏移
r = 0.0508       # 轮子半径

def draw_caster(ax, x, y, caster_id, wheel_angle=0):
    """
    绘制单个脚轮
    
    Args:
        ax: matplotlib轴对象
        x, y: 脚轮中心位置
        caster_id: 脚轮编号（1-4）
        wheel_angle: 轮子转向角度（弧度）
    """
    
    # 转向轴位置（脚轮中心）
    steer_center = np.array([x, y])
    
    # 接触点位置（考虑偏移）
    contact_offset = np.array([b_x * np.cos(wheel_angle) - b_y * np.sin(wheel_angle),
                              b_x * np.sin(wheel_angle) + b_y * np.cos(wheel_angle)])
    contact_point = steer_center + contact_offset
    
    # 绘制转向轴（小圆圈）
    steer_circle = patches.Circle(steer_center, 0.01, 
                                 facecolor='red', edgecolor='darkred', linewidth=2)
    ax.add_patch(steer_circle)
    
    # 绘制轮子（椭圆表示）
    wheel_width = r * 2
    wheel_length = 0.02
    
    # 轮子的方向向量
    wheel_direction = np.array([np.cos(wheel_angle), np.sin(wheel_angle)])
    wheel_perpendicular = np.array([-np.sin(wheel_angle), np.cos(wheel_angle)])
    
    # 轮子的四个角点
    wheel_corners = np.array([
        contact_point - wheel_direction * wheel_length/2 - wheel_perpendicular * wheel_width/2,
        contact_point + wheel_direction * wheel_length/2 - wheel_perpendicular * wheel_width/2,
        contact_point + wheel_direction * wheel_length/2 + wheel_perpendicular * wheel_width/2,
        contact_point - wheel_direction * wheel_length/2 + wheel_perpendicular * wheel_width/2
    ])
    
    # 绘制轮子
    wheel_patch = patches.Polygon(wheel_corners, 
                                 facecolor='lightblue', edgecolor='blue', linewidth=2)
    ax.add_patch(wheel_patch)
    
    # 绘制转向轴到接触点的连线
    ax.plot([steer_center[0], contact_point[0]], 
            [steer_center[1], contact_point[1]], 
            'k--', linewidth=1, alpha=0.7)
    
    # 标注脚轮编号
    ax.text(x + 0.03, y + 0.03, f'脚轮{caster_id}', 
            fontsize=12, fontweight='bold', 
            bbox=dict(boxstyle="round,pad=0.3", facecolor='yellow', alpha=0.7))
    
    # 标注电机ID
    steer_motor_id = 2 * caster_id - 1
    drive_motor_id = 2 * caster_id
    ax.text(x - 0.06, y - 0.03, f'转向电机ID:{steer_motor_id}', fontsize=8, color='red')
    ax.text(x - 0.06, y - 0.05, f'驱动电机ID:{drive_motor_id}', fontsize=8, color='blue')

def draw_base_outline(ax):
    """绘制底盘轮廓"""
    
    # 计算底盘边界
    margin = 0.05
    x_min, x_max = min(h_x) - margin, max(h_x) + margin
    y_min, y_max = min(h_y) - margin, max(h_y) + margin
    
    # 绘制矩形底盘轮廓
    base_rect = patches.Rectangle((x_min, y_min), x_max - x_min, y_max - y_min,
                                 linewidth=3, edgecolor='black', facecolor='lightgray', alpha=0.3)
    ax.add_patch(base_rect)
    
    # 标注底盘中心
    ax.plot(0, 0, 'ko', markersize=8)
    ax.text(0.02, 0.02, '底盘中心\n(0, 0)', fontsize=10, fontweight='bold',
            bbox=dict(boxstyle="round,pad=0.3", facecolor='white', alpha=0.8))

def add_dimensions(ax):
    """添加尺寸标注"""
    
    # x方向尺寸
    x_dim = abs(h_x[0] - h_x[2])
    ax.annotate('', xy=(h_x[0], -0.25), xytext=(h_x[2], -0.25),
                arrowprops=dict(arrowstyle='<->', color='green', lw=2))
    ax.text(0, -0.27, f'{x_dim:.3f}m', ha='center', fontsize=10, color='green', fontweight='bold')
    
    # y方向尺寸
    y_dim = abs(h_y[1] - h_y[0])
    ax.annotate('', xy=(-0.35, h_y[0]), xytext=(-0.35, h_y[1]),
                arrowprops=dict(arrowstyle='<->', color='green', lw=2))
    ax.text(-0.37, 0, f'{y_dim:.3f}m', ha='center', rotation=90, fontsize=10, color='green', fontweight='bold')
    
    # 脚轮偏移标注
    ax.text(0.25, -0.35, f'脚轮偏移参数:', fontsize=10, fontweight='bold')
    ax.text(0.25, -0.37, f'b_x = {b_x:.6f}m (前后偏移)', fontsize=9)
    ax.text(0.25, -0.39, f'b_y = {b_y:.6f}m (左右偏移)', fontsize=9)
    ax.text(0.25, -0.41, f'r = {r:.4f}m (轮子半径)', fontsize=9)

def add_coordinate_system(ax):
    """添加坐标系"""
    
    # 绘制坐标轴
    arrow_length = 0.1
    ax.arrow(0, 0, arrow_length, 0, head_width=0.01, head_length=0.01, 
             fc='red', ec='red', linewidth=2)
    ax.arrow(0, 0, 0, arrow_length, head_width=0.01, head_length=0.01, 
             fc='red', ec='red', linewidth=2)
    
    # 标注坐标轴
    ax.text(arrow_length + 0.02, -0.01, 'X', fontsize=12, fontweight='bold', color='red')
    ax.text(-0.01, arrow_length + 0.02, 'Y', fontsize=12, fontweight='bold', color='red')
    
    # 添加坐标系说明
    ax.text(-0.4, 0.35, '坐标系说明:', fontsize=10, fontweight='bold')
    ax.text(-0.4, 0.32, '• X轴：前进方向', fontsize=9)
    ax.text(-0.4, 0.29, '• Y轴：左侧方向', fontsize=9)
    ax.text(-0.4, 0.26, '• 原点：底盘几何中心', fontsize=9)

def create_layout_diagram():
    """创建底盘脚轮布置图"""
    
    fig, ax = plt.subplots(1, 1, figsize=(14, 10))
    
    # 绘制底盘轮廓
    draw_base_outline(ax)
    
    # 绘制坐标系
    add_coordinate_system(ax)
    
    # 绘制四个脚轮（不同的转向角度用于演示）
    wheel_angles = [0.2, -0.3, 0.1, -0.1]  # 演示不同的转向角度
    for i in range(4):
        draw_caster(ax, h_x[i], h_y[i], i+1, wheel_angles[i])
    
    # 添加尺寸标注
    add_dimensions(ax)
    
    # 设置图形属性
    ax.set_xlim(-0.45, 0.45)
    ax.set_ylim(-0.45, 0.4)
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    ax.set_xlabel('X (米)', fontsize=12)
    ax.set_ylabel('Y (米)', fontsize=12)
    ax.set_title('TidyBot2 底盘脚轮布置图\n4WIS/4WID (四轮独立转向驱动)', fontsize=16, fontweight='bold', pad=20)
    
    # 添加图例
    legend_elements = [
        patches.Circle((0, 0), 0.01, facecolor='red', label='转向轴中心'),
        patches.Rectangle((0, 0), 0.02, 0.1, facecolor='lightblue', label='驱动轮'),
        plt.Line2D([0], [0], color='black', linestyle='--', alpha=0.7, label='转向轴到接触点'),
        patches.Rectangle((0, 0), 0.1, 0.1, facecolor='lightgray', alpha=0.3, label='底盘轮廓')
    ]
    ax.legend(handles=legend_elements, loc='upper right', fontsize=10)
    
    # 添加技术参数表
    param_text = """技术参数:
• 控制频率: 250Hz
• 脚轮数量: 4个
• 电机总数: 8个 (TalonFX)
• 转向电机: 奇数ID (1,3,5,7)
• 驱动电机: 偶数ID (2,4,6,8)
• 最大速度: 0.5 m/s (线速度)
• 最大角速度: 1.57 rad/s
• 编码器: CANcoder (绝对编码器)"""
    
    ax.text(0.25, 0.35, param_text, fontsize=9, 
            bbox=dict(boxstyle="round,pad=0.5", facecolor='lightyellow', alpha=0.8),
            verticalalignment='top')
    
    plt.tight_layout()
    return fig, ax

def create_kinematics_diagram():
    """创建运动学示意图"""
    
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 8))
    
    # 左图：脚轮详细结构
    ax1.set_title('单个脚轮结构详图', fontsize=14, fontweight='bold')
    
    # 绘制单个脚轮的详细结构
    center = np.array([0, 0])
    angle = 0.3  # 转向角度
    
    # 转向轴
    steer_circle = patches.Circle(center, 0.02, facecolor='red', edgecolor='darkred', linewidth=2)
    ax1.add_patch(steer_circle)
    
    # 接触点
    contact_offset = np.array([b_x * np.cos(angle) - b_y * np.sin(angle),
                              b_x * np.sin(angle) + b_y * np.cos(angle)])
    contact_point = center + contact_offset
    
    # 轮子
    wheel_direction = np.array([np.cos(angle), np.sin(angle)])
    wheel_perpendicular = np.array([-np.sin(angle), np.cos(angle)])
    wheel_corners = np.array([
        contact_point - wheel_direction * 0.03 - wheel_perpendicular * r,
        contact_point + wheel_direction * 0.03 - wheel_perpendicular * r,
        contact_point + wheel_direction * 0.03 + wheel_perpendicular * r,
        contact_point - wheel_direction * 0.03 + wheel_perpendicular * r
    ])
    wheel_patch = patches.Polygon(wheel_corners, facecolor='lightblue', edgecolor='blue', linewidth=2)
    ax1.add_patch(wheel_patch)
    
    # 连接线
    ax1.plot([center[0], contact_point[0]], [center[1], contact_point[1]], 'k--', linewidth=2)
    
    # 标注
    ax1.plot([center[0], center[0] + 0.1], [center[1], center[1]], 'r-', linewidth=2)
    ax1.text(0.05, -0.02, 'X (前进)', fontsize=10, color='red')
    ax1.plot([center[0], center[0]], [center[1], center[1] + 0.1], 'r-', linewidth=2)
    ax1.text(-0.02, 0.05, 'Y (左侧)', fontsize=10, color='red')
    
    # 角度标注
    arc = patches.Arc(center, 0.1, 0.1, angle=0, theta1=0, theta2=np.degrees(angle), color='green', linewidth=2)
    ax1.add_patch(arc)
    ax1.text(0.06, 0.02, f'θ = {np.degrees(angle):.1f}°', fontsize=10, color='green')
    
    # 距离标注
    ax1.annotate('', xy=center, xytext=contact_point,
                arrowprops=dict(arrowstyle='<->', color='purple', lw=2))
    ax1.text(b_x/2 - 0.02, b_y/2 + 0.01, f'偏移\n({b_x:.3f}, {b_y:.3f})', 
             fontsize=9, color='purple', ha='center')
    
    ax1.set_xlim(-0.1, 0.1)
    ax1.set_ylim(-0.08, 0.12)
    ax1.set_aspect('equal')
    ax1.grid(True, alpha=0.3)
    ax1.set_xlabel('X (米)', fontsize=12)
    ax1.set_ylabel('Y (米)', fontsize=12)
    
    # 右图：运动学关系
    ax2.set_title('底盘运动学关系', fontsize=14, fontweight='bold')
    
    # 绘制简化的底盘和运动向量
    base_rect = patches.Rectangle((-0.15, -0.1), 0.3, 0.2, 
                                 linewidth=2, edgecolor='black', facecolor='lightgray', alpha=0.3)
    ax2.add_patch(base_rect)
    
    # 运动向量
    ax2.arrow(0, 0, 0.1, 0, head_width=0.02, head_length=0.02, fc='red', ec='red', linewidth=3)
    ax2.text(0.05, -0.04, 'vₓ', fontsize=12, color='red', ha='center')
    
    ax2.arrow(0, 0, 0, 0.08, head_width=0.02, head_length=0.02, fc='green', ec='green', linewidth=3)
    ax2.text(-0.03, 0.04, 'vᵧ', fontsize=12, color='green', ha='center')
    
    # 角速度
    circle = patches.Circle((0, 0), 0.05, fill=False, edgecolor='blue', linewidth=2)
    ax2.add_patch(circle)
    ax2.text(0.07, 0.07, 'ωz', fontsize=12, color='blue')
    
    # 运动学方程
    equations = """运动学关系:

dq = C × dx_local

其中:
• dq: 关节速度 [θ̇₁,φ̇₁,...,θ̇₄,φ̇₄]ᵀ
• dx_local: 本体速度 [vₓ,vᵧ,ωz]ᵀ  
• C: 运动学雅可比矩阵 (8×3)

里程计算法:
dx_local = C⁻¹ × dq
dx_global = R × dx_local"""
    
    ax2.text(0.2, 0.15, equations, fontsize=10,
             bbox=dict(boxstyle="round,pad=0.5", facecolor='lightyellow', alpha=0.8),
             verticalalignment='top')
    
    ax2.set_xlim(-0.2, 0.6)
    ax2.set_ylim(-0.15, 0.2)
    ax2.set_aspect('equal')
    ax2.grid(True, alpha=0.3)
    ax2.set_xlabel('X (米)', fontsize=12)
    ax2.set_ylabel('Y (米)', fontsize=12)
    
    plt.tight_layout()
    return fig, (ax1, ax2)

if __name__ == '__main__':
    # 创建并显示布置图
    fig1, ax1 = create_layout_diagram()
    plt.figure(fig1.number)
    plt.savefig('TidyBot2_底盘脚轮布置图.png', dpi=300, bbox_inches='tight')
    
    # 创建并显示运动学图
    fig2, (ax2, ax3) = create_kinematics_diagram()
    plt.figure(fig2.number)
    plt.savefig('TidyBot2_脚轮运动学示意图.png', dpi=300, bbox_inches='tight')
    
    plt.show()
    
    print("✅ 图表已生成并保存:")
    print("   - TidyBot2_底盘脚轮布置图.png")
    print("   - TidyBot2_脚轮运动学示意图.png") 