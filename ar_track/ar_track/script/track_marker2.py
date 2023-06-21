import rclpy, sys
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Twist
from ros2_aruco_interfaces.msg import ArucoMarkers
from math import degrees, radians, sqrt, sin, cos, pi, atan2
from tf_transformations import euler_from_quaternion #, quaternion_from_euler
from ar_track.move_tb3 import MoveTB3
import time
TARGET_ID = int(sys.argv[1]) # argv[1] = id of target marker
import numpy as np
from sensor_msgs.msg import Image, CompressedImage, LaserScan  # LaserScan 추가
from rclpy.qos import qos_profile_sensor_data
from math import radians, degrees, sqrt


# Turtlebot3 Specification
MAX_LIN_SPEED =  0.22
MAX_ANG_SPEED =  2.84

# make default speed of linear & angular
LIN_SPEED = MAX_LIN_SPEED * 0.075
ANG_SPEED = MAX_ANG_SPEED * 0.075

lin_spd = 0.0# 앞뒤 전진
ang_spd = 0.0# 좌우 회전

R = 1.5708


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
        #print("--------------------------")
        
        #print(min_ival)
        print(self.scan.ranges[0])
        #print("--------------------------")
        self.final_angle = min_ival
        
class PubCamera_MSG(Node):

    def __init__(self):
        super().__init__('pub_camera_msg')
        self.pub_camera = self.create_publisher(String, 'camera_msg', 10)
        self.camera_msg = String()
        
    def pub_camera_msg(self, camera_msg):
        msg = String()
        msg.data = camera_msg
        self.pub_camera.publish(msg)
        
class Sub_n_Pub(Node):
    def __init__(self):
        super().__init__('sub_n_pub')
        self.sub = self.create_subscription(String, 'Arrive_msg', self.get_msg, 10)
        self.pub = self.create_publisher(String, 'Finish_msg', 10)
        self.test_msg = String()
        self.str_msg = String()
        qos_profile = QoSProfile(depth=10)
        self.pub_cmd = self.create_publisher(Twist,'/cmd_vel',qos_profile)

    def get_msg(self, msg):
        self.str_msg = msg
        if self.str_msg.data == "Arrive":
            self.pub_test_msg("i am first turtlebot, go maker")
        elif self.str_msg.data == "stop":
            self.pub_test_msg("i am first turtlebot, i listen stop")

    def pub_test_msg(self, test_msg):
        msg = String()
        msg.data = test_msg
        self.pub.publish(msg)

class TrackMarker(Node):
    """   
                                                    ////////////| ar_marker |////////////
            y                      z                --------+---------+---------+--------
            ^  x                   ^                        |     R-0/|\R-0    R|
            | /                    |                        |       /0|0\       |
     marker |/                     | robot                  |      /  |  \      |
            +------> z    x <------+                        |     /   |   \     |
                                  /                         |  dist   |  dist   |
                                 /                          |   /     |     \   |
                                y                           |  /      |      \  |
                                                            | /       |       \0|
                                                            |/R-0    R|R    R-0\|
    pose.x = position.z                             (0 < O) x---------+---------x (0 > O)
    pose.y = position.x              [0]roll    (pos.x > O) ^                   ^ (pos.x < O)
    theta  = euler_from_quaternion(q)[1]pitch*              |                   |            
                                     [2]yaw               robot               robot
    """   
    def __init__(self):
        
        super().__init__('track_marker')
        qos_profile = QoSProfile(depth=10)
        
        self.sub_ar_pose  = self.create_subscription(
            ArucoMarkers,           # topic type
            'aruco_markers',        # topic name
            self.get_marker_pose_,  # callback function
            qos_profile)
            
        self.pub_tw   = self.create_publisher(Twist, '/cmd_vel', qos_profile)
        self.pub_lift = self.create_publisher(String, '/lift_ctrl_msg', qos_profile)
        self.timer    = self.create_timer(1, self.count_sec)
        
        self.pose = Pose()
        self.tw   = Twist()
        self.tb3  = MoveTB3()
        self.lift = String()
        
        self.theta   = 0.0
        self.dir     = 0
        self.th_ref  = 0.0
        self.z_ref   = 0.0
        self.cnt_sec = 0
        
        self.target_found = False
        
        
    def get_marker_pose_(self, msg):
        """
        orientation x,y,z,w ----+
                                +--4---> +-------------------------+
        input orientaion of marker-----> |                         |
                                         | euler_from_quaternion() |
        returnned rpy of marker <------- |                         |
                                +--3---- +-------------------------+
        r,p,y angle <-----------+
                                         +------------+------------+
                                         |   marker   |   robot    |
                                         +------------+------------+
          r: euler_from_quaternion(q)[0] | roll   (x) | (y) pitch  |
        * p: euler_from_quaternion(q)[1] | pitch  (y) | (z) yaw ** | <-- 
          y: euler_from_quaternion(q)[2] | yaw    (z) | (x) roll   | 
                                         +------------+------------+
        """
        if len(msg.marker_ids) == 0:    # no marker found
            self.target_found = False
        
        else: # if len(msg.marker_ids) != 0: # marker found at least 1EA
        
            for i in range(len(msg.marker_ids)):
            
                if msg.marker_ids[i] == TARGET_ID:  # target marker found
                    if self.target_found == False:
                        self.target_found = True                        
                    self.pose  = msg.poses[i]
                    self.theta = self.get_theta(self.pose)
                else:
                    self.target_found = False            
        
    def get_theta(self, msg):
        q = (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
        euler = euler_from_quaternion(q)
        theta = euler[1]        
        return theta
    
    def count_sec(self):
        self.cnt_sec = self.cnt_sec + 1    
        
    def pub_lift_ctrl(self, ctrl_msg):
        self.lift.data = ctrl_msg
        self.pub_lift.publish(self.lift)
    
    def stop_move(self):
        self.tw.linear.z = self.tw.angular.z = 0.0
        self.pub_tw.publish(self.tw)
        
    def publish_lift_ctrl(self, ctrl_msg):
        msg = String()
        msg = ctrl_msg
        self.pub_ctrl.publish(msg)        
        
        
def main(args=None):

    rclpy.init(args=args)
    node = TrackMarker()
    node2 = Sub_n_Pub()
    node3 = MoveTB3()
    node4 = SubLaser()
    node5 = PubCamera_MSG()
    node.tw.angular.z = 0.5 * ANG_SPEED
    target_5 = False
    go_msg = False
    lift_sts = False 
    Arrive_point = False
    pub_cmd = node3.create_publisher(Twist, '/cmd_vel',10)
    tw = Twist()
    
    try:
        rclpy.spin_once(node, timeout_sec=0.1)
        rclpy.spin_once(node2, timeout_sec=0.1)
        rclpy.spin_once(node3, timeout_sec=0.1)
        rclpy.spin_once(node4, timeout_sec=0.1)
        rclpy.spin_once(node5, timeout_sec=0.1)   
        while (1):
            print(node2.str_msg.data )
            rclpy.spin_once(node2, timeout_sec=0.1)
            if node2.str_msg.data == 'Arrive':
    
                
                Arrive_point = True
                if Arrive_point == True:
                    node5.pub_camera_msg('RE')   
                    node5.pub_camera_msg('LU')  
                    time.sleep(3)    
                    while rclpy.ok():
                    
                        print(node2.str_msg.data )
                        if node.theta != 0.0:   break   # this means target marker found
                        node.pub_tw.publish(node.tw)
                        rclpy.spin_once(node, timeout_sec=0.1)
                        
                    
                    node.stop_move()
                    print("\n----- 1_target marker found!\n") ###########################
                    
                    
                    target_5 = True
                    go_msg = True
                    print(go_msg)
                    print(target_5)
                    while rclpy.ok():
                        rclpy.spin_once(node, timeout_sec=0.1)
                        rclpy.spin_once(node2, timeout_sec=0.1)
                        rclpy.spin_once(node3, timeout_sec=0.1)
                        rclpy.spin_once(node4, timeout_sec=0.1)
                        rclpy.spin_once(node5, timeout_sec=0.1)
                        
                        if go_msg == True:
                            if len(node4.scan.ranges) > 0:  # 배열이 비어있지 않을 경우
                                if node4.scan.ranges[0] != 0:
                                    if node4.scan.ranges[0] < 0.30:# cm만 수정
                                        lin_spd = 0.00
                                        tw.linear.x = lin_spd
                                        pub_cmd.publish(tw)
                                        time.sleep(3)
                                        node5.pub_camera_msg('open')
                                        rclpy.spin_once(node5, timeout_sec=0.1)
                                        lift_sts = True 
                                        if lift_sts == True: 
                                            time.sleep(3)
                                            lin_spd = -0.05
                                            tw.linear.x = lin_spd
                                            pub_cmd.publish(tw)
                                            time.sleep(3)
                                            
                                            lin_spd = 0.00
                                            tw.linear.x = lin_spd
                                            pub_cmd.publish(tw)
                                            node5.pub_camera_msg('LD')
                                            sys.exit(0) # 리프트 다운하기 위하여 
                                            break
                                        ##리프트 업, 오픈, 다운 추가
                            
                                    else:
                                        lin_spd = 0.01
                                        tw.linear.x = lin_spd
                                        pub_cmd.publish(tw)
            else:
                lin_spd = 0.00
                ang_spd = 0.00
                tw.angular.z = ang_spd
                tw.linear.x = lin_spd
                pub_cmd.publish(tw) 
                rclpy.spin_once(node, timeout_sec=0.1)
                rclpy.spin_once(node2, timeout_sec=0.1)
                rclpy.spin_once(node3, timeout_sec=0.1)
                rclpy.spin_once(node4, timeout_sec=0.1)
                rclpy.spin_once(node5, timeout_sec=0.1)   
                
            #pub_cmd.publish(tw)
            
        '''
        if(target_5 == True):
            lin_spd = 0.0
            ang_spd = 0.0
            rclpy.spin_once(node, timeout_sec=0.1)
            tw.angular.x = lin_spd
            tw.angular.z = ang_spd
            pub_cmd.publish(tw)
            go_msg = True
            if(go_msg == True):
                lin_spd = 0.05  
                rclpy.spin_once(node, timeout_sec=0.1)
                tw.angular.x = lin_spd 
                pub_cmd.publish(tw)'''
            
                 
          
        
        
        
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt(SIGINT)')
        
    finally:
        node.destroy_node()
        node2.destroy_node()
        node3.destroy_node()
        node4.destroy_node()
        node5.destroy_node()
        rclpy.shutdown()
    
            
if __name__ == '__main__':
    main()

