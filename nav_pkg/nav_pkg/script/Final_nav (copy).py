from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped

from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from nav2_msgs.action import FollowWaypoints
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
import time
import rclpy

class ClientFollowPoints(Node):
    def __init__(self):
        super().__init__('client_follow_points')
        self._client = ActionClient(self, FollowWaypoints, '/FollowWaypoints')
        self._result = None

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
        self._result = result

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback

    def get_result(self):
        return self._result

def main(args=None):
    rclpy.init(args=args)

    follow_points_client = ClientFollowPoints()
    amcl_pose_subscriber = AmclPoseSubscriber()
    executor = SingleThreadedExecutor()

    waypoints = [
        # Define your waypoints here
        # Example: (x, y, orientation)
        (1.95, -0.393, 1.0)
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


    while True:
        current_pose = amcl_pose_subscriber.get_current_pose()
        if current_pose is not None:
            print('AMCL Pose: x={0}, y={1}, orientation={2}'.format(
                current_pose.position.x, current_pose.position.y, current_pose.orientation.w))

        follow_points_client.send_points(poses)
        executor.spin_once()

        time.sleep(0.1)  # Wait for a small duration between iterations

    follow_points_client.destroy_node()
    amcl_pose_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

