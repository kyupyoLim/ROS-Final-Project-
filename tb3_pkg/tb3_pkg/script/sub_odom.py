import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from nav_msgs.msg import Odometry
#from math import radians, degrees, pi

class SubOdom(Node):

    def __init__(self):    
        super().__init__('sub_tb3_odom')
        qos_profile = QoSProfile(depth=10)
        
        self.sub = self.create_subscription(
            Odometry,       # topic type
            'odom',         # topic name
            self.get_odom, # callback function
            qos_profile)        
        self.odom = Odometry()
    def get_odom(self, msg):
        self.odom = msg
        self.get_logger().info('x = "%s", y = "%s"' %(self.odom.pose.pose.position.x, self.odom.pose.pose.position.y))
        
        


def main(args=None):
    rclpy.init(args=args)
    node = SubOdom()
    
    try:
        rclpy.spin(node)
                
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt(SIGINT)')
        
    finally:
        node.destroy_node()
        rclpy.shutdown()
    
            
if __name__ == '__main__':
    main()


