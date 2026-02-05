#!/usr/bin/env python3
import numpy as np
import math

class OmniKinematics:
    def __init__(self, wheel_radius=0.05, wheel_distance=0.15):
        """
        四轮正交全向轮运动学
        :param wheel_radius: 轮子半径 (米)
        :param wheel_distance: 轮子到底盘中心的距离 (米)
        """
        self.wheel_radius = wheel_radius
        self.wheel_distance = wheel_distance
        
        # 轮子角度 (弧度) - 45度正交布局
        # 前进方向与x轴夹角
        self.wheel_angles = [
            np.pi/4,      # 轮1: +45度 (右前)
            3*np.pi/4,    # 轮2: +135度 (左前)
            -np.pi/4,     # 轮3: -45度 (右后)
            -3*np.pi/4    # 轮4: -135度 (左后)
        ]
        
        # 轮子方向向量 (单位向量)
        self.wheel_directions = []
        for angle in self.wheel_angles:
            self.wheel_directions.append([
                math.cos(angle),
                math.sin(angle)
            ])
    
    def inverse_kinematics(self, vx, vy, omega):
        """
        逆运动学：将底盘速度转换为四个轮子的线速度
        :param vx: x方向线速度 (m/s)
        :param vy: y方向线速度 (m/s) 
        :param omega: 角速度 (rad/s)
        :return: 四个轮子的线速度 (m/s)
        """
        # 使用运动学转换矩阵
        # 根据参考代码的思路，构建转换矩阵
        # 顺序为: 右前、左前、右后、左后
        l = self.wheel_distance  # 轮子到中心的距离
        
        # 构建转换矩阵，对应于右前、左前、右后、左后四个轮子
        # 每行代表一个轮子的运动学方程系数 [vx_coeff, vy_coeff, omega_coeff]
        conversion_matrix = np.array([
            [1/np.sqrt(2),  1/np.sqrt(2), -l],  # 右前轮 (+45度)
            [1/np.sqrt(2), -1/np.sqrt(2),  l],  # 左前轮 (+135度)
            [1/np.sqrt(2), -1/np.sqrt(2), -l],  # 右后轮 (-45度)
            [1/np.sqrt(2),  1/np.sqrt(2),  l]   # 左后轮 (-135度)
        ])
        
        # 机器人运动向量 [vx, vy, omega]
        motion_vector = np.array([vx, -(vy), omega])
        
        # 计算每个轮子的线速度 (m/s)
        wheel_linear_speeds = conversion_matrix.dot(motion_vector)
        
        return wheel_linear_speeds.tolist()
    
    def rpm_to_mps(self, rpm):
        """RPM 转换为 m/s"""
        return rpm * 2 * math.pi * self.wheel_radius / 60
    
    def mps_to_rpm(self, mps):
        """m/s 转换为 RPM"""
        return mps * 60 / (2 * math.pi * self.wheel_radius)















# #!/usr/bin/env python3
# import numpy as np
# import math

# class OmniKinematics:
#     def __init__(self, wheel_radius=0.05, wheel_distance=0.15):
#         """
#         四轮正交全向轮运动学
#         :param wheel_radius: 轮子半径 (米)
#         :param wheel_distance: 轮子到底盘中心的距离 (米)
#         """
#         self.wheel_radius = wheel_radius
#         self.wheel_distance = wheel_distance
        
#         # 轮子角度 (弧度) - 45度正交布局
#         # 前进方向与x轴夹角
#         self.wheel_angles = [
#             np.pi/4,      # 轮1: +45度
#             3*np.pi/4,    # 轮2: +135度
#             -np.pi/4,     # 轮3: -45度
#             -3*np.pi/4    # 轮4: -135度
#         ]
        
#         # 轮子方向向量 (单位向量)
#         self.wheel_directions = []
#         for angle in self.wheel_angles:
#             self.wheel_directions.append([
#                 math.cos(angle),
#                 math.sin(angle)
#             ])
    
#     def inverse_kinematics(self, vx, vy, omega):
#         """
#         逆运动学：将底盘速度转换为四个轮子的线速度
#         :param vx: x方向线速度 (m/s)
#         :param vy: y方向线速度 (m/s) 
#         :param omega: 角速度 (rad/s)
#         :return: 四个轮子的线速度 (m/s)
#         """
#         wheel_velocities = []
        
#         for i, direction in enumerate(self.wheel_directions):
#             # 轮子速度 = 底盘速度在轮子方向上的投影 + 旋转分量
#             # v_wheel = v_robot · direction + ω × r
#             linear_component = vx * direction[0] + vy * direction[1]
            
#             # 旋转速度分量：ω × r_perpendicular
#             # 对于正交轮，旋转分量是ω × wheel_distance
#             # 方向取决于轮子位置
#             rotational_component = omega * self.wheel_distance

#             if i == 0:  # 轮1: +45度
#                 rotational_component = omega * self.wheel_distance
#                 wheel_speed = linear_component + rotational_component

#             elif i == 1:  # 轮2: +135度
#                 rotational_component = -omega * self.wheel_distance  # 反向
#                 wheel_speed = -(linear_component + rotational_component)
#             elif i == 2:  # 轮3: -45度
#                 rotational_component = omega * self.wheel_distance
#                 wheel_speed = linear_component + rotational_component

#             elif i == 3:  # 轮4: -135度
#                 rotational_component = -omega * self.wheel_distance  # 反向
#                 wheel_speed = -(linear_component + rotational_component)



#             # 总轮子速度
#             # wheel_speed = linear_component + rotational_component
#             wheel_velocities.append(wheel_speed)
        
#         return wheel_velocities
    
#     def rpm_to_mps(self, rpm):
#         """RPM 转换为 m/s"""
#         return rpm * 2 * math.pi * self.wheel_radius / 60
    
#     def mps_to_rpm(self, mps):
#         """m/s 转换为 RPM"""
#         return mps * 60 / (2 * math.pi * self.wheel_radius)