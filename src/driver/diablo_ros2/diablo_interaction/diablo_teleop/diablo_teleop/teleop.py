# #!/usr/bin/env python3
# from http.client import OK
# import rclpy
# import time
# import sys
# import tty
# import termios
# import threading
# from rclpy.node import Node
# from motion_msgs.msg import MotionCtrl
# from geometry_msgs.msg import Twist

# print("Teleop start now!")
# print("Press '`' to exit!")

# keyQueue = []
# ctrlMsgs = MotionCtrl()
# old_setting = termios.tcgetattr(sys.stdin)

# def readchar():
#     fd = sys.stdin.fileno()
#     old_settings = termios.tcgetattr(fd)
#     try:
#         tty.setraw(sys.stdin.fileno())
#         ch = sys.stdin.read(1)
#     finally:
#         termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
#     return ch

# def getKeyBoard():
#     global keyQueue
#     while True:
#         c = readchar()
#         keyQueue.append(c)


# t1 =threading.Thread(target=getKeyBoard)
# t1.setDaemon(True)
# t1.start()


# def generMsgs(forward=None,left=None,roll=None,up=None,
#                 pitch=None,mode_mark=False,height_ctrl_mode = None,
#                 pitch_ctrl_mode = None,roll_ctrl_mode = None,stand_mode = None,
#                 jump_mode = False,dance_mode = None):
#     global ctrlMsgs

#     ctrlMsgs.mode_mark = mode_mark
#     ctrlMsgs.mode.jump_mode = jump_mode

#     if dance_mode is not None:
#         ctrlMsgs.mode.split_mode = dance_mode
#     if forward is not None:
#         ctrlMsgs.value.forward = forward
#     if left is not None:
#         ctrlMsgs.value.left = left
#     if pitch is not None:
#         ctrlMsgs.value.pitch = pitch
#     if roll is not None:
#         ctrlMsgs.value.roll = roll
#     if up is not None:
#         ctrlMsgs.value.up = up
#     if height_ctrl_mode is not None:
#         ctrlMsgs.mode.height_ctrl_mode = height_ctrl_mode
#     if pitch_ctrl_mode is not None:
#         ctrlMsgs.mode.pitch_ctrl_mode = pitch_ctrl_mode
#     if roll_ctrl_mode is not None:
#         ctrlMsgs.mode.roll_ctrl_mode = roll_ctrl_mode
#     if stand_mode is not None:
#         ctrlMsgs.mode.stand_mode = stand_mode


# def main(args=None):
#     global ctrlMsgs
#     rclpy.init(args=args) 
#     node = Node("diablo_teleop_node")  
#     cmd_vel_subscription = node.create_subscription(
#     Twist,
#     '/cmd_vel',
#     cmd_vel_callback,  # 需要定义这个回调函数
#     10)

#     linear = Twist.linear.x
#     angular = Twist.angular.z

#     teleop_cmd = node.create_publisher(MotionCtrl,"diablo/MotionCmd",2)

#     try:
#         generMsgs(forward=linear)
#         generMsgs(left=angular)
#         teleop_cmd.publish(ctrlMsgs)
#     except KeyboardInterrupt:
#         pass

#     cmd_vel_subscriber.destroy_node()
#     rclpy.shutdown()




#     # while True:
#     #     if len(keyQueue) > 0:
#     #         key = keyQueue.pop(0)
#     #         if key == 'w':
#     #             generMsgs(forward=1.0)
#     #         elif key == 's':
#     #             generMsgs(forward=-1.0)
#     #         elif key == 'a':
#     #             generMsgs(left=1.0)
#     #         elif key == 'd':
#     #             generMsgs(left=-1.0)
#     #         elif key == 'e':
#     #             generMsgs(roll=0.1)
#     #         elif key == 'q':
#     #             generMsgs(roll=-0.1)
#     #         elif key == 'r':
#     #             generMsgs(roll=0.0)

#     #         elif key == 'h':
#     #             generMsgs(up = -0.5)
#     #         elif key == 'j':
#     #             generMsgs(up = 1.0)
#     #         elif key == 'k':
#     #            generMsgs(up = 0.5)
#     #         elif key == 'l':
#     #            generMsgs(up = 0.0)
                
#     #         elif key == 'u':
#     #             generMsgs(pitch = 0.5)
#     #         elif key == 'i':
#     #             generMsgs(pitch = 0.0)
#     #         elif key == 'o':
#     #             generMsgs(pitch = -0.5)

#     #         elif key == 'v':
#     #             generMsgs(mode_mark=True,height_ctrl_mode=True)
#     #         elif key == 'b':
#     #             generMsgs(mode_mark=True,height_ctrl_mode=False)
#     #         elif key == 'n':
#     #             generMsgs(mode_mark=True,pitch_ctrl_mode=True)
#     #         elif key == 'm':
#     #             generMsgs(mode_mark=True,pitch_ctrl_mode=False)

#     #         elif key == 'z':
#     #             generMsgs(mode_mark=True,stand_mode=True)
#     #             teleop_cmd.publish(ctrlMsgs)
#     #             generMsgs(up=1.0)
#     #             teleop_cmd.publish(ctrlMsgs)
#     #         elif key == 'x':
#     #             generMsgs(mode_mark=True,stand_mode=False)
#     #             teleop_cmd.publish(ctrlMsgs)
#     #         elif key == 'c':
#     #             generMsgs(mode_mark=True,jump_mode=True)
#     #             teleop_cmd.publish(ctrlMsgs)
#     #         elif key == 'f':
#     #             generMsgs(mode_mark=True,dance_mode=True)
#     #             teleop_cmd.publish(ctrlMsgs)
#     #         elif key == 'g':
#     #             generMsgs(mode_mark=True,dance_mode=False)
#     #             teleop_cmd.publish(ctrlMsgs)
#     #         elif key == '`':
#     #             break
#         # else:
#         #     ctrlMsgs.mode_mark = False
#         #     ctrlMsgs.mode.split_mode = False
#         #     ctrlMsgs.value.forward = 0.0
#         #     ctrlMsgs.value.left = 0.0
            
            
#     #     teleop_cmd.publish(ctrlMsgs)
#     #     time.sleep(0.04)
#     # termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_setting)
#     # print('exit!')
#     # rclpy.shutdown() 


#!/usr/bin/env python3
from http.client import OK
import rclpy
import time
import sys
import tty
import termios
import threading
from rclpy.node import Node
from motion_msgs.msg import MotionCtrl
from geometry_msgs.msg import Twist

print("Teleop start now!")
print("Press '`' to exit!")

keyQueue = []
ctrlMsgs = MotionCtrl()
old_setting = termios.tcgetattr(sys.stdin)

def readchar():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

def getKeyBoard():
    global keyQueue
    while True:
        c = readchar()
        keyQueue.append(c)


t1 =threading.Thread(target=getKeyBoard)
t1.setDaemon(True)
t1.start()

def cmd_vel_callback(msg):
    # Process velocity commands from /cmd_vel topic
    linear_x = msg.linear.x
    angular_z = msg.angular.z
    
    # Update the robot control messages with received velocities
    generMsgs(forward=linear_x, left=angular_z)

def generMsgs(forward=None,left=None,roll=None,up=None,
                pitch=None,mode_mark=False,height_ctrl_mode = None,
                pitch_ctrl_mode = None,roll_ctrl_mode = None,stand_mode = None,
                jump_mode = False,dance_mode = None):
    global ctrlMsgs

    ctrlMsgs.mode_mark = mode_mark
    ctrlMsgs.mode.jump_mode = jump_mode

    if dance_mode is not None:
        ctrlMsgs.mode.split_mode = dance_mode
    if forward is not None:
        ctrlMsgs.value.forward = forward
    if left is not None:
        ctrlMsgs.value.left = left
    if pitch is not None:
        ctrlMsgs.value.pitch = pitch
    if roll is not None:
        ctrlMsgs.value.roll = roll
    if up is not None:
        ctrlMsgs.value.up = up
    if height_ctrl_mode is not None:
        ctrlMsgs.mode.height_ctrl_mode = height_ctrl_mode
    if pitch_ctrl_mode is not None:
        ctrlMsgs.mode.pitch_ctrl_mode = pitch_ctrl_mode
    if roll_ctrl_mode is not None:
        ctrlMsgs.mode.roll_ctrl_mode = roll_ctrl_mode
    if stand_mode is not None:
        ctrlMsgs.mode.stand_mode = stand_mode


# ctrlMsgs.value.forward
# ctrlMsgs.value.left
# teleop_cmd.publish(ctrlMsgs)

def main(args=None):
    global ctrlMsgs
    rclpy.init(args=args) 
    node = Node("diablo_teleop_node")  
    
    # Create subscription to cmd_vel topic
    cmd_vel_subscription = node.create_subscription(
        Twist,
        '/cmd_vel',
        cmd_vel_callback,
        10
    )

    teleop_cmd = node.create_publisher(MotionCtrl,"diablo/MotionCmd",2)

    try:
        rclpy.spin(teleop_cmd)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

    # while True:
    #     if len(keyQueue) > 0:
    #         key = keyQueue.pop(0)
    #         if key == 'w':
    #             generMsgs(forward=1.0)
    #         elif key == 's':
    #             generMsgs(forward=-1.0)
    #         elif key == 'a':
    #             generMsgs(left=1.0)
    #         elif key == 'd':
    #             generMsgs(left=-1.0)
    #         elif key == 'e':
    #             generMsgs(roll=0.1)
    #         elif key == 'q':
    #             generMsgs(roll=-0.1)
    #         elif key == 'r':
    #             generMsgs(roll=0.0)

    #         elif key == 'h':
    #             generMsgs(up = -0.5)
    #         elif key == 'j':
    #             generMsgs(up = 1.0)
    #         elif key == 'k':
    #            generMsgs(up = 0.5)
    #         elif key == 'l':
    #            generMsgs(up = 0.0)
                
    #         elif key == 'u':
    #             generMsgs(pitch = 0.5)
    #         elif key == 'i':
    #             generMsgs(pitch = 0.0)
    #         elif key == 'o':
    #             generMsgs(pitch = -0.5)

    #         elif key == 'v':
    #             generMsgs(mode_mark=True,height_ctrl_mode=True)
    #         elif key == 'b':
    #             generMsgs(mode_mark=True,height_ctrl_mode=False)
    #         elif key == 'n':
    #             generMsgs(mode_mark=True,pitch_ctrl_mode=True)
    #         elif key == 'm':
    #             generMsgs(mode_mark=True,pitch_ctrl_mode=False)

    #         elif key == 'z':
    #             generMsgs(mode_mark=True,stand_mode=True)
    #             teleop_cmd.publish(ctrlMsgs)
    #             generMsgs(up=1.0)
    #             teleop_cmd.publish(ctrlMsgs)
    #         elif key == 'x':
    #             generMsgs(mode_mark=True,stand_mode=False)
    #             teleop_cmd.publish(ctrlMsgs)
    #         elif key == 'c':
    #             generMsgs(mode_mark=True,jump_mode=True)
    #             teleop_cmd.publish(ctrlMsgs)
    #         elif key == 'f':
    #             generMsgs(mode_mark=True,dance_mode=True)
    #             teleop_cmd.publish(ctrlMsgs)
    #         elif key == 'g':
    #             generMsgs(mode_mark=True,dance_mode=False)
    #             teleop_cmd.publish(ctrlMsgs)
    #         elif key == '`':
    #             break
        # else:
        #     ctrlMsgs.mode_mark = False
        #     ctrlMsgs.mode.split_mode = False
        #     ctrlMsgs.value.forward = 0.0
        #     ctrlMsgs.value.left = 0.0
            
            
    #     teleop_cmd.publish(ctrlMsgs)
    #     time.sleep(0.04)
    # termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_setting)
    # print('exit!')
    # rclpy.shutdown() 