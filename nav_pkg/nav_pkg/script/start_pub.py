import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from start_pub.getchar import Getchar

class PubCatch_MSG(Node):

    def __init__(self):
        super().__init__('pub_Catch_msg')
        self.pub_Catch = self.create_publisher(String, 'Catch_msg', 10)
        self.Catch_msg = String()
        
    def pub_Catch_msg(self, Catch_msg):
        msg = String()
        msg.data = Catch_msg
        self.pub_Catch.publish(msg)
        
def main(args=None):
    rclpy.init(args=args)

    node = PubCatch_MSG()

    #rclpy.spin(node)
    try:
        kb = Getchar()
        key =''
        while rclpy.ok():
            key = kb.getch()
            if key == '1':
                node.pub_Catch_msg('Catch')
            else:
                pass
    except KeyboardInterrupt:
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
