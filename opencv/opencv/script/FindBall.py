import rclpy 
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2


class ImageConvertor(Node):
  def __init__(self):
    super().__init__('image_subscriber')
    self.subscription = self.create_subscription(CompressedImage, 
                'camera/image/compressed', 
                self.get_compressed, 
                10) 
    self.img_pub = self.create_publisher(Image, 'image_gray', 10)
    self.subscription # prevent unused variable warning
    
    # Used to convert between ROS and OpenCV images
    self.bridge = CvBridge()
 

   
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


    #frame = cv2.flip(frame,1)
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
        cv2.circle(self.cv_img, (x, y), 1, (0, 100, 100), 3)  # 원 중심출

        # 원 중심 좌표를 이용해 사각형을 그리기
        x1, y1 = x - r, y - r
        x2, y2 = x + r, y + r
        cv2.rectangle(self.cv_img, (x1, y1), (x2, y2), (0, 255, 0), 5)
        prevCircle = chosen

        # 거리 측정
        ball_pixels = 2 * r  # 탁구공 지름의 이미지 상 크기 (픽셀)
        ball_distance = (BALL_DIAMETER * CAMERA_WIDTH) / (2 * ball_pixels * np.tan(np.deg2rad(CAMERA_FOV / 2)))  # 탁구공과 카메라 사이의 거리 (mm)
        ball_distance = int(ball_distance) / 10 #cm 환산
        print(str(ball_distance) + 'cm')


        img_msg = self.bridge.cv2_to_imgmsg(self.cv_img)        
        self.img_pub.publish(img_msg)

        cv2.imshow("FindBall", self.cv_img)    
        cv2.waitKey(1)
 
def main(args=None):
  global lin_spd
  # Initialize the rclpy library
  rclpy.init(args=args)
  
  # Create the node
  node = ImageConvertor()

  # Spin the node so the callback function is called.
  rclpy.spin(node)
  
  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  image_subscriber.destroy_node()
  
  # Shutdown the ROS client library for Python
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()
