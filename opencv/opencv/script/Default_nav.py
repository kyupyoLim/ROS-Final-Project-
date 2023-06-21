import rclpy, sys, time
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage, LaserScan  # LaserScan 추가
from cv_bridge import CvBridge
import cv2
from rclpy.qos import QoSProfile
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from opencv.move_tb3 import MoveTB3
from math import radians, degrees, sqrt, atan2



lin_spd = 0.0  # 앞뒤 전진
ang_spd = 0.0  # 좌우 회전
ball_distance = 0

min_ival = 0

class SubLaser(Node):
    
    def __init__(self):
        super().__init__('sub_laser')
        #qos_profile = QoSProfile(depth=10)
        
        # define subscriber
        self.sub_scan = self.create_subscription(
           LaserScan,           # topic type
            '/scan',        # topic name
            self.get_scan,   # callback function
            qos_profile_sensor_data)
        self.scan = LaserScan()
        self.final_angle = min_ival
      
    def get_scan(self, msg):
        self.scan = msg
        
        min_val = 5
        Detect_Distance = 0.3
        for i in range(360) :
        	if(self.scan.ranges[i]<=min_val):
        		if(self.scan.ranges[i]!=0):
        			min_val = self.scan.ranges[i]
        			min_ival = i
        
        '''
        for i in range(360):
            if self.scan.ranges[i] <= min_val: 
                if self.scan.ranges[i] != 0:
                    min_val = self.scan.ranges[i]
                    min_ival = i
        
        if min_val <= Detect_Distance:
            if 0 <= min_ival < 90:
                print("Obstacle detected from 0 to 90 degrees.")
            elif 90 <= min_ival < 180:
                print("Obstacle detected from 90 to 180 degrees.")
            elif 180 <= min_ival < 270:
                print("Obstacle detected from 180 to 270 degrees.")
            elif 270 <= min_ival < 360:
                print("Obstacle detected within 270 to 360 degrees.")
        #print("--------------------------")'''
        
        #print(min_ival)
        print(self.scan.ranges[0])
        #print("--------------------------")
        self.final_angle = min_ival
          
class SubBall_MSG(Node):

    def __init__(self):
        super().__init__('sub_Ball_msg')
        qos_profile = QoSProfile(depth=10)
        self.subscription = self.create_subscription(
            String, '/ball_detection', self.get_Ball_msg, qos_profile)
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
    node3 = MoveTB3()
    node4 = SubBall_MSG()
    node5 = SubLaser()
    #rclpy.spin(node)
  
    pub = node3.create_publisher(Twist, '/cmd_vel', 10)  # 수정된 부분
    tw = Twist()

  
    try:

        while rclpy.ok():
            rclpy.spin_once(node3, timeout_sec=0.1)
            rclpy.spin_once(node4, timeout_sec=0.1)
            rclpy.spin_once(node5, timeout_sec=0.1)

            print("Checking condition:", node4.str_msg.data)
            if node4.str_msg.data == 'True':
                print('Find Ball!')
                rclpy.spin_once(node4, timeout_sec=0.1)
                lin_spd = 0.00
                ang_spd = 0.00
                tw.linear.x = lin_spd
                tw.angular.z = ang_spd
                pub.publish(tw)
                sys.exit(0)  ######중요########

     
                pub.publish(tw)
            elif node4.str_msg.data == 'False':
                print('Not Find Ball')
                if len(node5.scan.ranges) > 0:  # 배열이 비어있지 않을 경우
                
                #쓰레기값 버리기 라이더센서
                #회전 90도 angular_z로
                
                    if node5.scan.ranges[0] != 0:
                        if node5.scan.ranges[0] < 0.35 and node5.scan.ranges[0] > 0.30 :
                            lin_spd = 0.00
                            tw.linear.x = lin_spd
                            pub.publish(tw)
                            node3.rotate(radians(-90))
                            
                            
                        else:
                            lin_spd = 0.02
                            tw.linear.x = lin_spd
                            pub.publish(tw)

                rclpy.spin_once(node4, timeout_sec=0.1) 

            
            
                

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    finally:
        node3.destroy_node()
        node4.destroy_node()
        node5.destroy_node()
        # Shutdown the ROS client library for Python
        rclpy.shutdown()
  
if __name__ == '__main__':
    main()

