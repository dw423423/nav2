from tf2_ros import Buffer,TransformListener
import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped,Pose
import tf_transformations 
import time
import math

class PoseNode(BasicNavigator):
    def __init__(self,node_name="pose_node"):
        super().__init__(node_name)

        self.declare_parameter('initial_point', [0.0,0.0,0.0])
        self.declare_parameter('goal_point', [0.0,0.0,0.0])
        self.initial_point = self.get_parameter('initial_point').value
        self.goal_point = self.get_parameter('goal_point').value
        self.buff_ = Buffer()
        self.listener = TransformListener(self.buff_,self)

    def get_pose_by_xyyaw(self,x,y,yaw):

        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = x
        pose.pose.position.y = y

        quat = tf_transformations.quaternion_from_euler(0,0,yaw)
        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]
        return pose
    
    def init_robot_pose(self):
        self.get_logger().info(f'kaishi: ()')
        self.initial_point = self.get_parameter('initial_point').value
        init_pose = self.get_pose_by_xyyaw(self.initial_point[0],self.initial_point[1],self.initial_point[2])
        self.setInitialPose(init_pose)
        self.waitUntilNav2Active()


    def get_target_points(self):
        
        points = []
        self.target_points_ = self.get_parameter('goal_point').value
        for index in range(int(len(self.target_points_)/3)):
            x,y,yaw = self.target_points_[index*3],self.target_points_[index*3+1],self.target_points_[index*3+2]
            points.append([x,y,yaw])
            self.get_logger().info(f'添加目标点: ({x},{y},{yaw})')
        return points
    
    def nav_to_pose(self,target_point):
        
        self.goToPose(target_point)
        while not self.isTaskComplete():
            feedback = self.getFeedback()
            self.get_logger().info(f'剩余距离：{feedback.distance_remaining}')
            self.get_logger().info(f'任务是否完成：{self.isTaskComplete()}')

            time.sleep(1)
        result = self.getResult()
        self.get_logger().info(f'导航结果：{result}')

    def get_current_pose(self):
        
        while rclpy.ok():
            try:
                result = self.buff_.lookup_transform('map','base_footprint',
                                                    rclpy.time.Time(seconds=0),rclpy.time.Duration(seconds=1))
                self.get_logger().info(f'当前位置: ({result.transform.translation}')
        
            except:
                self.get_logger().error('获取当前位置失败')
    
def main():
    rclpy.init()
    partol = PoseNode()       

    # partol.init_robot_pose()



    while rclpy.ok():
        points = partol.get_target_points()
        for point in points:
            x,y,yaw = point[0],point[1],point[2]
            target_pose = partol.get_pose_by_xyyaw(x,y,yaw)
            partol.nav_to_pose(target_pose)
    rclpy.shutdown()

if __name__ == '__main__':
    main()