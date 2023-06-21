import rclpy, sys
from rclpy.node import Node
from rclpy.qos import QoSProfile

from geometry_msgs.msg import Twist
from turtle_pkg.getchar import Getchar
MAX_LIN_SPD = 0.22
MIN_LIN_SPD = -0.22
MAX_ANG_SPD = 2.84
MIN_ANG_SPD = -2.84
lin_spd = 0.0
ang_spd = 0.0
LIN_STEP = 0.01
ANG_STEP = 0.04
msg = """    forward
              +---+
              | w |
          +---+---+---+
turn left | a | s | d | turn left
          +---+---+---+
              | x | 
              +---+   
             backward
             
### space for stop\n
"""

class RemoteTb3(Node):

    def __init__(self):
        self.cnt_sec = 0
        super().__init__('remote_tb3')
        qos_profile = QoSProfile(depth=10)
        #self.pub = self.create_publisher(Twist, '/cmd_vel', qos_profile)
        self.timer    = self.create_timer(1, self.count_sec)

    def count_sec(self):
        self.cnt_sec = self.cnt_sec + 1
        #print(self.cnt_sec)


def main(args=None):
    global lin_spd, ang_spd, MAX_LIN_SPD, MIN_LIN_SPD, MAX_ANG_SPD, MIN_ANG_SPD
    rclpy.init(args=args)
    node= RemoteTb3()
    pub = node.create_publisher(Twist, '/cmd_vel', 10)
    tw = Twist()
    kb = Getchar()
    key = ' '
    count = 0
    print(msg)
    try:
            while rclpy.ok():
                key = kb.getch()
                if      key == 'w':
                        if lin_spd + LIN_STEP <= MAX_LIN_SPD:
                            lin_spd = lin_spd + LIN_STEP
                        else:
                            lin_spd = MAX_LIN_SPD
                elif key == 'x':
                        if lin_spd - LIN_STEP >= MIN_LIN_SPD:
                            lin_spd = lin_spd - LIN_STEP
                        else:
                            lin_spd = MIN_LIN_SPD
                elif      key == 'a':
                        if ang_spd + ANG_STEP <= MAX_ANG_SPD:
                            ang_spd = ang_spd + ANG_STEP
                        else:
                            ang_spd = MAX_ANG_SPD
                elif key == 'd':
                        if ang_spd - ANG_STEP >= MIN_ANG_SPD:
                            ang_spd = ang_spd - ANG_STEP
                        else:
                            ang_spd = MIN_ANG_SPD
                elif key == ' ':
                    lin_spd = ang_spd = 0.0
                elif key == 's':
                    lin_spd = ang_spd = 0.0
                
                
                tw.linear.x = lin_spd
                tw.angular.z = ang_spd
                pub.publish(tw)
                node.get_logger().info('linear.x = "%s", angular.z = "%s"' %(lin_spd, ang_spd))
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt(SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()
        
if __name__ == '__main__':
    main()
