# Author: Jimmy Wu
# Date: October 2024
#
# 逆运动学求解器
#
# 功能：
#   - 基于雅可比矩阵的数值 IK 求解
#   - 带阻尼最小二乘法
#   - 零空间优化（趋向 retract 姿态）
#
# 算法：
#   - Damped Least Squares (DLS) IK
#   - Nullspace projection（零空间投影）
#
# 参考资料：
#   - https://github.com/bulletphysics/bullet3/blob/master/examples/ThirdPartyLibs/BussIK/Jacobian.cpp
#   - https://github.com/kevinzakka/mjctrl/blob/main/diffik_nullspace.py
#   - https://github.com/google-deepmind/dm_control/blob/main/dm_control/utils/inverse_kinematics.py

import mujoco
import numpy as np

# 阻尼系数（防止雅可比矩阵奇异）
DAMPING_COEFF = 1e-12

# 每次迭代的最大角度变化（防止大幅跳跃）
MAX_ANGLE_CHANGE = np.deg2rad(45)  # 45度

class IKSolver:
    """
    逆运动学求解器
    
    功能：
        - 给定末端位姿，计算关节角度
        - 使用阻尼最小二乘法迭代求解
        - 零空间优化趋向 retract 姿态
    
    特点：
        - 快速：每次求解约 0.6 ms
        - 稳定：阻尼最小二乘防止奇异
        - 自然：零空间优化保持舒适姿态
    
    示例：
        ik_solver = IKSolver(ee_offset=0.12)  # 0.12m 为夹爪长度
        pos = np.array([0.45, 0.0, 0.4])      # 目标位置
        quat = np.array([0.707, 0.707, 0, 0]) # 目标姿态
        qpos = ik_solver.solve(pos, quat, current_qpos)
    """
    def __init__(self, ee_offset=0.0):
        """
        初始化 IK 求解器
        
        参数:
            ee_offset: 末端执行器偏移量（通常为夹爪长度），单位: 米
        """
        # 加载机械臂模型（不含夹爪）
        self.model = mujoco.MjModel.from_xml_path('models/kinova_gen3/gen3.xml')
        self.data = mujoco.MjData(self.model)
        self.model.body_gravcomp[:] = 1.0  # 启用重力补偿

        # 缓存常用引用
        self.qpos0 = self.model.key('retract').qpos  # retract 姿态作为零空间目标
        self.site_id = self.model.site('pinch_site').id
        self.site_pos = self.data.site(self.site_id).xpos  # 末端位置
        self.site_mat = self.data.site(self.site_id).xmat  # 末端旋转矩阵

        # 添加末端执行器偏移（用于夹爪）
        # 0.061525 来自 Kinova URDF 中的夹爪安装距离
        self.model.site(self.site_id).pos = np.array([0.0, 0.0, -0.061525 - ee_offset])

        # 预分配数组（避免重复分配内存）
        self.err = np.empty(6)  # 位姿误差 [位置误差(3), 旋转误差(3)]
        self.err_pos, self.err_rot = self.err[:3], self.err[3:]
        self.site_quat = np.empty(4)
        self.site_quat_inv = np.empty(4)
        self.err_quat = np.empty(4)
        self.jac = np.empty((6, self.model.nv))  # 雅可比矩阵
        self.jac_pos, self.jac_rot = self.jac[:3], self.jac[3:]
        self.damping = DAMPING_COEFF * np.eye(6)  # 阻尼矩阵
        self.eye = np.eye(self.model.nv)  # 单位矩阵

    def solve(self, pos, quat, curr_qpos, max_iters=20, err_thresh=1e-4):
        """
        求解逆运动学
        
        参数:
            pos: 目标位置 (x, y, z)，单位: 米
            quat: 目标姿态四元数 (x, y, z, w)
            curr_qpos: 当前关节角度（作为初始猜测）
            max_iters: 最大迭代次数（默认 20）
            err_thresh: 误差阈值，达到后停止迭代（默认 1e-4）
        
        返回:
            qpos: 关节角度 (7,)，单位: 弧度
        
        算法流程:
            1. 从当前关节角度开始
            2. 迭代：
               a. 计算位姿误差（位置 + 旋转）
               b. 计算雅可比矩阵
               c. 使用阻尼最小二乘法计算更新量
               d. 添加零空间优化（趋向 retract 姿态）
               e. 限制最大角度变化
               f. 更新关节角度
            3. 返回求解结果
        """
        # 四元数格式转换：(x, y, z, w) -> (w, x, y, z)
        quat = quat[[3, 0, 1, 2]]

        # 设置机械臂初始关节配置
        self.data.qpos = curr_qpos

        # 迭代求解
        for _ in range(max_iters):
            # 更新末端位姿（正运动学）
            mujoco.mj_kinematics(self.model, self.data)
            mujoco.mj_comPos(self.model, self.data)

            # 计算位置误差
            self.err_pos[:] = pos - self.site_pos

            # 计算旋转误差
            mujoco.mju_mat2Quat(self.site_quat, self.site_mat)  # 当前姿态
            mujoco.mju_negQuat(self.site_quat_inv, self.site_quat)  # 逆四元数
            mujoco.mju_mulQuat(self.err_quat, quat, self.site_quat_inv)  # 误差四元数
            mujoco.mju_quat2Vel(self.err_rot, self.err_quat, 1.0)  # 转换为角速度误差

            # 检查是否达到目标位姿
            if np.linalg.norm(self.err) < err_thresh:
                break

            # 计算关节角度更新量
            mujoco.mj_jacSite(self.model, self.data, self.jac_pos, self.jac_rot, self.site_id)
            
            # 阻尼最小二乘法：update = J^T (JJ^T + λI)^{-1} err
            update = self.jac.T @ np.linalg.solve(self.jac @ self.jac.T + self.damping, self.err)
            
            # 零空间优化：趋向 retract 姿态
            # update += (I - J^+ J) (q_retract - q_current)
            qpos0_err = np.mod(self.qpos0 - self.data.qpos + np.pi, 2 * np.pi) - np.pi
            update += (self.eye - (self.jac.T @ np.linalg.pinv(self.jac @ self.jac.T + self.damping)) @ self.jac) @ qpos0_err

            # 限制最大角度变化（防止大幅跳跃）
            update_max = np.abs(update).max()
            if update_max > MAX_ANGLE_CHANGE:
                update *= MAX_ANGLE_CHANGE / update_max

            # 应用更新
            mujoco.mj_integratePos(self.model, self.data.qpos, update, 1.0)

        return self.data.qpos.copy()

if __name__ == '__main__':
    ik_solver = IKSolver()
    home_pos, home_quat = np.array([0.456, 0.0, 0.434]), np.array([0.5, 0.5, 0.5, 0.5])
    retract_qpos = np.deg2rad([0, -20, 180, -146, 0, -50, 90])

    import time
    start_time = time.time()
    for _ in range(1000):
        qpos = ik_solver.solve(home_pos, home_quat, retract_qpos)
    elapsed_time = time.time() - start_time
    print(f'Time per call: {elapsed_time:.3f} ms')  # 0.59 ms

    # Home: 0, 15, 180, -130, 0, 55, 90
    print(np.rad2deg(ik_solver.solve(home_pos, home_quat, retract_qpos)).round())
