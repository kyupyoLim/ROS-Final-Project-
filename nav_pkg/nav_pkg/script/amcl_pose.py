from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

class AmclPoseRepublisher(Node):
    def __init__(self):
        super().__init__('amcl_pose_republisher')
        self.publisher_ = self.create_publisher(PoseStamped, 'amcl_pose_republished', 10)
        self.subscription_ = self.create_subscription(
            PoseWithCovarianceStamped,
            'amcl_pose',
            self.amcl_pose_callback,
            QoSProfile(depth=10)
        )

    def amcl_pose_callback(self, msg):
        pose = msg.pose.pose

        republished_pose = PoseStamped()
        republished_pose.header = msg.header
        republished_pose.pose = pose

        self.publisher_.publish(republished_pose)

        self.get_logger().info('Current Pose: x={0}, y={1}, orientation={2}'.format(
            pose.position.x, pose.position.y, pose.orientation.w))

def main(args=None):
    rclpy.init(args=args)
    amcl_pose_republisher = AmclPoseRepublisher()

    try:
        rclpy.spin(amcl_pose_republisher)
    except KeyboardInterrupt:
        pass

    amcl_pose_republisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

