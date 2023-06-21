from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from nav2_msgs.action import FollowWaypoints
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
import rclpy
from std_msgs.msg import Bool
from rclpy.qos import QoSProfile
from std_msgs.msg import String

import sys

class ClientFollowPoints(Node):
    def __init__(self):
        super().__init__('client_follow_points')
        self._client = ActionClient(self, FollowWaypoints, '/FollowWaypoints')

    def send_points(self, points):
        msg = FollowWaypoints.Goal()
        msg.poses = points

        self._client.wait_for_server()
        self._send_goal_future = self._client.send_goal_async(msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.missed_waypoints))

        if result.missed_waypoints == 0:
            self.get_logger().info('All waypoints reached')  # 모든 웨이포인트를 돌았을 때 출력

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback.current_waypoint))

        if feedback.status.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Waypoint reached: {0}'.format(feedback.current_waypoint))  # 각 waypoint 도착 시 출력



class SubCatch_MSG(Node):

    def __init__(self):
        super().__init__('sub_Catch_msg')
        qos_profile = QoSProfile(depth=10)
        self.subscription = self.create_subscription(
            String, '/Catch_msg', self.get_Catch_msg, qos_profile )
        self.Catch_msg = String()
        
        self.test_msg = String()
        self.str_msg = String()


                    
    def get_Catch_msg(self, msg):
        self.str_msg = msg
        #print("Received message:", self.str_msg.data)  # 메시지 출력 확인
        if self.str_msg.data == "Catch":
            pass

        
        #print(self.led_msg)
        
class PubArrive_MSG(Node):

    def __init__(self):
        super().__init__('pub_Arrive_msg')
        self.pub_Arrive = self.create_publisher(String, 'Arrive_msg', 10)
        self.Arrive_msg = String()
        
    def pub_led_msg(self, Arrive_msg):
        msg = String()
        msg.data = Arrive_msg
        self.pub_Arrive.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    follow_points_client = ClientFollowPoints()
    executor = SingleThreadedExecutor()
    node4 = SubCatch_MSG()
    node5 = PubArrive_MSG()
    rclpy.spin_once(follow_points_client,timeout_sec=0.1)
    rclpy.spin_once(node4, timeout_sec=0.1)
    rclpy.spin_once(node5, timeout_sec=0.1)
    
    waypoints = [
        # Define your waypoints here
        # Example: (x, y, orientation)
        (-0.0119,-0.00441,1.0),
        ## ////////////////////////여기에 추가///////////////////
        # Add more waypoints as needed
    ]

    poses = []
    for waypoint in waypoints:
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = rclpy.time.Time().to_msg()
        pose.pose.position.x = waypoint[0]
        pose.pose.position.y = waypoint[1]
        pose.pose.orientation.w = waypoint[2]
        poses.append(pose)
    
    
    
    while rclpy.ok():
        print("Checking condition:", node4.str_msg.data)
        rclpy.spin_once(follow_points_client,timeout_sec=0.1)
        rclpy.spin_once(node4, timeout_sec=0.1)
        rclpy.spin_once(node5, timeout_sec=0.1)
        move_count = 0
        
        if node4.str_msg.data == 'Catch':
            print('Go to basket')
            rclpy.spin_once(node4, timeout_sec=0.1)
            follow_points_client.send_points(poses)
            executor.spin_once()
            move_count = 1
            if(move_count == 1):
            ########Publisher#########
                node5.pub_Catch_msg('Arrive')
            
            

    follow_points_client.destroy_node()
    node4.destroy_node()
    node5.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

