#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class TestVelocity(Node):
    def __init__(self):
        super().__init__('test_velocity')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        time.sleep(1)
        
    def test_movement(self):
        # 测试向前移动
        self.send_velocity(0.0, 0.0, 0.1, 0.0)
        

    
    def send_velocity(self, vx, vy, omega, duration):
        msg = Twist()
        msg.linear.x = vx
        msg.linear.y = vy
        msg.angular.z = omega
        
        self.get_logger().info(f'发送速度命令: vx={vx}, vy={vy}, ω={omega}, 持续{duration}秒')
        while 1:
            self.publisher.publish(msg)
        
        # start_time = time.time()
        # while time.time() - start_time < duration:
        #     self.publisher.publish(msg)
        #     time.sleep(0.1)
        
        # # 停止
        # msg.linear.x = 0.0
        # msg.linear.y = 0.0
        # msg.angular.z = 0.0
        # self.publisher.publish(msg)
        # time.sleep(0.5)

def main():
    rclpy.init()
    node = TestVelocity()
    node.test_movement()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()