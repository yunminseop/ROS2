import rclpy as rp
from rclpy.node import Node

from turtlesim.msg import Pose

class Individual(Node):

    def __init__(self):
        super().__init__("individual_node")
        self.subscription = self.create_subscription(Pose, '/turtle1/pose', self.callback, 10)
        self.subscription

    def callback(self, msg):
        print("X:", msg.x, ", Y:", msg.y)


def main(args=None):
    rp.init(args=args)

    individual = Individual()
    rp.spin(individual)

    individual.destroy_node()
    rp.shutdown()


if __name__ == "__main__":
    main()