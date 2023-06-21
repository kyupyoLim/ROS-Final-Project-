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



class ImageConvertor(Node):
  
  def __init__(self):
    super().__init__('image_subscriber')
    self.subscription = self.create_subscription(CompressedImage, 
                'camera/image/compressed', self.get_compressed, 10) 
    self.img_pub = self.create_publisher(Image, 'image_gray', 10)
    qos_profile = QoSProfile(depth=10)
    self.pub = self.create_publisher(Twist, '/cmd_vel', qos_profile)
    self.subscription # prevent unused variable warning
    
    # Used to convert between ROS and OpenCV images
    self.bridge = CvBridge()
    self.Final_Distance = 0
    self.X_Mid_Val = 0 # x 화면 중앙점
    self.Y_Mid_Val = 0 # y 화면 중앙점
    self.Ball_X_Val = 0 #객체의 중앙
    self.Ball_Y_Val = 0



   

   
  def get_compressed(self, msg):
    self.cv_img = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
    
    prevCircle = None
    dist = lambda x1, y1, x2, y2: np.sqrt((x1-x2)**2+(y1-y2)**2)

    # 카메라 해상도
    CAMERA_WIDTH = 320
    CAMERA_HEIGHT = 240
    
    # 탁구공 지름 (mm)
    BALL_DIAMETER = 40

    # 카메라 화각 (도)
    CAMERA_FOV = 90


    #cv_img = cv2.flip(cv_img,1)
    # 이미지를 HSV 색상공간으로 변환
    hsv = cv2.cvtColor(self.cv_img, cv2.COLOR_BGR2HSV)

    # V 채널에 대해 히스토그램 평활화 수행
    h, s, v = cv2.split(hsv)
    v = cv2.equalizeHist(v)
    hsv = cv2.merge([h, s, v])

    # 주황색 범위
    lower_orange = np.array([0, 100, 100])
    upper_orange = np.array([30, 255, 255])

    # 주황색 범위 내의 원을 찾음
    mask1 = cv2.inRange(hsv, lower_orange, upper_orange)
    mask2 = cv2.medianBlur(mask1, 5)  # 미디언 필터 추가
    blur = cv2.GaussianBlur(mask2, (5, 5), 0)
    circles = cv2.HoughCircles(blur, cv2.HOUGH_GRADIENT, 1, 100, param1=80, param2=20, minRadius=5, maxRadius=200)

    # 이미지의 중심점 구하기
    center_x = int(CAMERA_WIDTH/ 2)#320/2 = 160
    center_y = int(CAMERA_HEIGHT / 2)#240/2 = 120

    if circles is not None:
        circles = np.int32(np.around(circles))
        chosen = None
        for i in circles[0, :]:
            if chosen is None:
                chosen = i
            if prevCircle is not None:
                if dist(chosen[0], chosen[1], prevCircle[0], prevCircle[1]) <= dist(i[0], i[1], prevCircle[0],prevCircle[1]):
                    chosen = i

        x, y, r = chosen
        cv2.circle(self.cv_img, (x, y), 1, (0, 100, 100), 3)  # 원 중심 추출

        # 원 중심 좌표를 이용해 사각형을 그리기
        x1, y1 = x - r, y - r
        x2, y2 = x + r, y + r
        cv2.rectangle(self.cv_img, (x1, y1), (x2, y2), (0, 255, 0), 5)
        prevCircle = chosen

        # 거리 측정
        ball_pixels = 2 * r  # 탁구공 지름의 이미지 상 크기 (픽셀)
        ball_distance = (BALL_DIAMETER * CAMERA_WIDTH) / (2 * ball_pixels * np.tan(np.deg2rad(CAMERA_FOV / 2)))  # 탁구공과 카메라 사이의 거리 (mm)
        ball_distance = int(ball_distance) / 10 #cm 환산
        self.X_Mid_Val = center_x
        self.Y_Mid_Val = center_y 
        self.Ball_X_Val = x
        self.Ball_Y_Val = y
        self.Final_Distance = ball_distance

        #print(str(self.dist) + 'cm')
        
        
        img_msg = self.bridge.cv2_to_imgmsg(self.cv_img)        
        self.img_pub.publish(img_msg)
        

        cv2.imshow("FindBall", self.cv_img)    
        cv2.waitKey(1)
        
        
        
class PubCamera_MSG(Node):

    def __init__(self):
        super().__init__('pub_camera_msg')
        self.pub_camera = self.create_publisher(String, 'camera_msg', 10)
        self.camera_msg = String()
        
    def pub_camera_msg(self, camera_msg):
        msg = String()
        msg.data = camera_msg
        self.pub_camera.publish(msg)


def main(args=None):
  # Initialize the rclpy library
  rclpy.init(args=args)
  
  # Create the node
  node = ImageConvertor()
  node2 = PubCamera_MSG()
  node3 = MoveTB3()
  #rclpy.spin(node)
  
  pub = node.create_publisher(Twist, '/cmd_vel', 10)
  tw = Twist()
 
  
  try:
    #좌우 회전부
    # 검출된 객체가 화면 중앙에 위치해 있는지, 좌우에 위치해 있는지 판단
    while True:
        
        print('now_val : ' + str(node.Ball_X_Val) + ',' + 'mid_val : ' + str(node.X_Mid_Val))
        node2.pub_camera_msg('UP')
        node2.pub_camera_msg('open')
        if node.Ball_X_Val > node.X_Mid_Val + 10 :#오른쪽에 있다고 판단   320 > x > 180
            ang_spd = -0.02                       #왼쪽으로 회전 -0.02
            rclpy.spin_once(node, timeout_sec=0.1)
            tw.angular.z = ang_spd
            pub.publish(tw)
        elif node.Ball_X_Val < node.X_Mid_Val - 10 and  0 < node.Ball_X_Val :#왼쪽에 있다고 판단  0 < x < 140
            ang_spd = 0.02                          #오른쪽으로 회전 0.02    
            rclpy.spin_once(node, timeout_sec=0.1)
            tw.angular.z = ang_spd
            pub.publish(tw)

        elif node.Ball_X_Val == 0 and node.X_Mid_Val ==0:
            ang_spd = 0.00                            
            rclpy.spin_once(node, timeout_sec=0.1)
            tw.angular.z = ang_spd
            pub.publish(tw)
            node2.pub_camera_msg('UP')

        else:#멈춤
            ang_spd = 0.00
            rclpy.spin_once(node, timeout_sec=0.1) 
            tw.angular.z = ang_spd
            pub.publish(tw)
            break      

        #앞뒤 전진부
        while True:

            if node.Final_Distance > 4:#전진 0.02
                lin_spd = 0.02
                rclpy.spin_once(node, timeout_sec=0.1)
                tw.linear.x = lin_spd
                pub.publish(tw)
                if node.Y_Mid_Val < node.Ball_Y_Val:
                    node2.pub_camera_msg('DW')
            elif (node.Final_Distance < 4)and(node.Final_Distance > 0):#후진 -0.02
                lin_spd = -0.02   
                rclpy.spin_once(node, timeout_sec=0.1)
                tw.linear.x = lin_spd
                pub.publish(tw)
            elif node.Final_Distance == 0:
                lin_spd = 0.00
                rclpy.spin_once(node, timeout_sec=0.1) 
                tw.linear.x = lin_spd
                pub.publish(tw)
                node2.pub_camera_msg('UP')
            elif node.Final_Distance == 4:#멈춤
                lin_spd = 0.00
                rclpy.spin_once(node, timeout_sec=0.1) 
                tw.linear.x = lin_spd
                pub.publish(tw)
                node2.pub_camera_msg('close')
                if(0<node.Ball_Y_Val<240 and 0<node.Ball_X_Val<320):#잡았을 때 #open : 0, close : 1 // default = 0
                    #angle = radians(float(180))
                    #dist = float(0)

                    #node3.rotate(angle)
                    #node3.straight(dist)
                    
                    node2.pub_camera_msg('UP')
                    break
      
                else:#못 잡았을 때
                    node2.pub_camera_msg('open')
                    node2.pub_camera_msg('UP')
      
            print('distance : '+str(node.Final_Distance)) 
  
    
    
    
  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  finally:
    node.destroy_node()
  # Shutdown the ROS client library for Python
    rclpy.shutdown()
  
if __name__ == '__main__':
  main()
