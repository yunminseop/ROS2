import rclpy as rp
from rclpy.node import Node

from geometry_msgs.msg import Twist

class Commander(Node):
    def __init__(self):
        super().__init__("commander_node")
        self.commander = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = 1.0
        msg.linear.y = 1.0
        msg.angular.z = 1.0

        self.commander.publish(msg)

    
def main(args=None):
    rp.init(args = args)

    commander = Commander()
    rp.spin(commander)

    commander.destroy_node()
    rp.shutdown()


if __name__ == "__main__":
    main()