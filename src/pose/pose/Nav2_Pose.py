# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import PoseWithCovarianceStamped
# from nav2_simple_commander.robot_navigator import BasicNavigator
# from std_msgs.msg import Header

# class RobustInitialPosePublisher(Node):
#     def __init__(self):
#         super().__init__('robust_initial_pose_publisher')
#         self.publisher_ = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        
#         # 控制发布的参数
#         self.publish_count = 0
#         self.max_count = 5  # 你想发布的总次数
#         self.publish_period = 1.0  # 发布间隔，单位：秒 (频率不宜过高)
        
#         # 创建一个定时器，每隔 publish_period 秒调用一次 timer_callback
#         self.timer = self.create_timer(self.publish_period, self.timer_callback)
#         self.get_logger().info('节点已启动，将开始周期性发布初始位姿...')
        
#     def timer_callback(self):
#         """定时器回调函数，每次被调用时发布一次位姿"""
#         if self.publish_count < self.max_count:
#             self.publish_initial_pose()
#             self.publish_count += 1
#             self.get_logger().info(f'发布第 {self.publish_count} 次初始位姿')
#         else:
#             # 达到发布次数后，取消定时器
#             self.get_logger().info('已完成指定次数的发布，即将关闭节点。')
#             self.timer.cancel()  # 停止定时器
#             # 可以选择让节点自动退出，或保持运行但不发布
#             # rclpy.shutdown()  # 直接关闭，但通常由外部控制
            
#     def publish_initial_pose(self):
#         # 构建消息内容 (与之前相同)
#         msg = PoseWithCovarianceStamped()
#         msg.header = Header()
#         msg.header.stamp = self.get_clock().now().to_msg()
#         msg.header.frame_id = 'map'
#         msg.pose.pose.position.x = 0.353
#         msg.pose.pose.position.y = 0.619
#         msg.pose.pose.position.z = 0.005
#         msg.pose.pose.orientation.x = 0.0
#         msg.pose.pose.orientation.y = 0.001
#         msg.pose.pose.orientation.z = 1.0
#         msg.pose.pose.orientation.w = -0.019
#         # 协方差矩阵
#         msg.pose.covariance[0] = 0.25   # x方差
#         msg.pose.covariance[7] = 0.25   # y方差
#         msg.pose.covariance[35] = 0.068 # yaw方差
        
#         self.publisher_.publish(msg)

# def main(args=None):
#     rclpy.init(args=args)
#     node = RobustInitialPosePublisher()
    
#     try:
#         # 保持节点运行，直到定时器停止或收到终止信号
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()




# 根据视频写的初始化初始化位姿发布节点

#!/usr/bin/env python3
"""
使用nav2_simple_commander初始化机器人位置
"""

import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import tf_transformations
import time



def create_pose_setamped(navigator: BasicNavigator,x, y,yaw, frame_id='map'):
    q_x, q_y, q_z, q_w = tf_transformations.quaternion_from_euler(0.0, 0.0, yaw)
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = 0.0
    pose.pose.orientation.x = q_x
    pose.pose.orientation.y = q_y
    pose.pose.orientation.z = q_z
    pose.pose.orientation.w = q_w
    return pose

    
def main():

    rclpy.init()
    nav = BasicNavigator()


    # 创建初始位姿消息
    # initial_pose = PoseStamped()
    # initial_pose.header.frame_id = 'map'
    # initial_pose.header.stamp = nav.get_clock().now().to_msg()
    
    # # 设置位置
    # q_x, q_y, q_z, q_w = tf_transformations.quaternion_from_euler(0, 0, 0)    
    # initial_pose.pose.position.x = 0
    # initial_pose.pose.position.y = 0
    # initial_pose.pose.position.z = 0
    # initial_pose.pose.orientation.x = q_x
    # initial_pose.pose.orientation.y = q_y
    # initial_pose.pose.orientation.z = q_z
    # initial_pose.pose.orientation.w = q_w
    # nav.setInitialPose(initial_pose)


    initial_pose = create_pose_setamped(nav,0.0,0.0,0.0,'map')

    nav.setInitialPose(initial_pose)# 设置初始位姿

    # 等待导航服务启动 
    nav.waitUntilNav2Active()

    #--Set goal pose
    goal_pose1 = create_pose_setamped(nav,1.0,0.0,0.0,'map')
    goal_pose2 = create_pose_setamped(nav,2.0,0.0,0.0,'map')
    goal_pose3 = create_pose_setamped(nav,3.0,2.0,1.57,'map')


    #--Go to one goal pose
    # nav.goToPose(goal_pose)
    # while not nav.isTaskComplete():  
    #     feedback = nav.getFeedback()
    #     # print(feedback)
    # print(nav.getResult())

    #--follow multiple goal poses
    waypoints = [goal_pose1,goal_pose2,goal_pose3]
    nav.followWaypoints(waypoints)
    while not nav.isTaskComplete():  
        feedback = nav.getFeedback()
        # print(feedback)
    print(nav.getResult())
    

    rclpy.shutdown()
     

                
   
if __name__ == '__main__':
    main()