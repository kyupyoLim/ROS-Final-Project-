import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped 
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from nav2_msgs.action import FollowWaypoints
from std_msgs.msg import String
# from rclpy.duration import Duration # Handles time for ROS 2

class ClientFollowPoints(Node):

    def __init__(self):
        self.goal_now = ''
        self.goal_prv = ''
        self.goal_changed = False
        super().__init__('client_follow_points')
        self.subscription = self.create_subscription(
            String, '/goal_msg', self.get_goal_msg, 10 )
        self._client = ActionClient(self, FollowWaypoints, '/FollowWaypoints')

    def get_goal_msg(self, msg):
        self.goal_now = msg.data
        if self.goal_now != self.goal_prv:
            self.goal_prv = self.goal_now
            self.goal_changed = True
        print(self.goal_now)
    
    
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

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback.current_waypoint))

def main(args=None):
    rclpy.init(args=args)

    client = ClientFollowPoints()
    print('client initialized')

    goal0 = PoseStamped()
    goal0.header.frame_id = "map"
    goal0.header.stamp.sec = 0
    goal0.header.stamp.nanosec = 0
    goal0.pose.position.z = 0.0
    goal0.pose.position.x = 0.113
    goal0.pose.position.y = 0.252
    goal0.pose.orientation.w = 1.0
    
    goal1 = PoseStamped()
    goal1.header.frame_id = "map"
    goal1.header.stamp.sec = 0
    goal1.header.stamp.nanosec = 0
    goal1.pose.position.z = 0.0
    goal1.pose.position.x = 1.71 ###### goal1_x
    goal1.pose.position.y = -1.23 ###### goal1_y
    goal1.pose.orientation.w = 1.0
    
    goal2 = PoseStamped()
    goal2.header.frame_id = "map"
    goal2.header.stamp.sec = 0
    goal2.header.stamp.nanosec = 0
    goal2.pose.position.z = 0.0
    goal2.pose.position.x = 1.71 ###### goal2_x
    goal2.pose.position.y = 2.58 ###### goal2_y
    goal2.pose.orientation.w = 1.0
    
    goal3 = PoseStamped()
    goal3.header.frame_id = "map"
    goal3.header.stamp.sec = 0
    goal3.header.stamp.nanosec = 0
    goal3.pose.position.z = 0.0
    goal3.pose.position.x = 2.83 ###### goal3_x
    goal3.pose.position.y = 2.58 ###### goal3_y
    goal3.pose.orientation.w = 1.0
    
    goal4 = PoseStamped()
    goal4.header.frame_id = "map"
    goal4.header.stamp.sec = 0
    goal4.header.stamp.nanosec = 0
    goal4.pose.position.z = 0.0
    goal4.pose.position.x = 2.83 ###### goal4_x
    goal4.pose.position.y = -1.23 ###### goal4_y
    goal4.pose.orientation.w = 1.0
    
    
    try:    
        while rclpy.ok():
            rclpy.spin_once(client)
            if client.goal_changed is True:
                if client.goal_now == 'goal1':
                    target = [goal0]                    
                    client.send_points(mgoal)
                    
                elif client.goal_now == 'goal2':
                    target = [goal1]
                    client.send_points(mgoal)
                elif client.goal_now == 'goal3':
                    target = [goal1]
                print("new target point is %s") %(client.goal_now)
                client.send_points(target)
                
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt(SIGINT)')
        
    finally:
        client.destroy_node()
        rclpy.shutdown()
