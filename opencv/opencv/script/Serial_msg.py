import rclpy, sys
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from opencv.move_tb3 import MoveTB3
from math import radians, degrees, sqrt, atan2

lin_spd = 0.0#앞뒤 전진
ang_spd = 0.0#좌우 회전
ball_distance = 0



        
class PubCamera_MSG(Node):

    def __init__(self):
        super().__init__('pub_camera_msg')
        self.pub_camera = self.create_publisher(String, 'camera_msg', 10)
        self.camera_msg = String()
        
    def pub_camera_msg(self, camera_msg):
        msg = String()
        msg.data = camera_msg
        self.pub_camera.publish(msg)
        
class SubBall_MSG(Node):

    def __init__(self):
        super().__init__('sub_Ball_msg')
        qos_profile = QoSProfile(depth=10)
        self.subscription = self.create_subscription(
            String, '/ball_detection', self.get_Ball_msg, qos_profile )
        self.Ball_msg = String()
        
        self.test_msg = String()
        self.str_msg = String()


                    
    def get_Ball_msg(self, msg):
        self.str_msg = msg
        #print("Received message:", self.str_msg.data)  # 메시지 출력 확인
        if self.str_msg.data == "True":
            pass
        elif self.str_msg.data == "False":
            pass
        
        #print(self.led_msg)


def main(args=None):
  # Initialize the rclpy library
  rclpy.init(args=args)
  
  # Create the node
  #node = ImageConvertor()
  node2 = PubCamera_MSG()
  node3 = MoveTB3()
  node4 = SubBall_MSG()
  #rclpy.spin(node)
  
  #pub = node.create_publisher(Twist, '/cmd_vel', 10)
  #tw = Twist()
  rclpy.spin_once(node4, timeout_sec=0.1)
  rclpy.spin_once(node2, timeout_sec=0.1)
  count = 0
  
  try:
    #좌우 회전부
    # 검출된 객체가 화면 중앙에 위치해 있는지, 좌우에 위치해 있는지 판단
    while rclpy.ok():
        node2.pub_camera_msg('RE')
        
        #node2.pub_camera_msg('open')
        #node2.pub_camera_msg('close')
        
        #node2.pub_camera_msg('UP')
        #node2.pub_camera_msg('DW')
        
        #node2.pub_camera_msg('CT')
        
        
        
        #node2.pub_camera_msg('LD')
        #node2.pub_camera_msg('LU')
        
        
        
        count+=1
        print(count)
        rclpy.spin_once(node2, timeout_sec=0.1)
        if(count==20):
            break


  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  finally:
    #node.destroy_node()
    node2.destroy_node()
    node3.destroy_node()
    node4.destroy_node()
  # Shutdown the ROS client library for Python
    rclpy.shutdown()
  
if __name__ == '__main__':
  main()
