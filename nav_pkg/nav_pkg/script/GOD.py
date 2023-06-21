from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from nav2_msgs.action import FollowWaypoints
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
import rclpy

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

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback.current_waypoint))

def main(args=None):
    rclpy.init(args=args)

    follow_points_client = ClientFollowPoints()
    executor = SingleThreadedExecutor()

    waypoints = [
        # Define your waypoints here
        # Example: (x, y, orientation)

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
        follow_points_client.send_points(poses)
        executor.spin_once()
        rclpy.spin_once(follow_points_client, timeout_sec=1.0)  # Process a single event with a timeout of 1 second

    follow_points_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

