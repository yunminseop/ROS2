import rclpy as rp
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from central_control.commander import Commander
from central_control.individual import Individual

def main(args=None):
    rp.init()

    sub = Individual()
    pub = Commander()

    executor = MultiThreadedExecutor()

    executor.add_node(sub)
    executor.add_node(pub)

    try:
        executor.spin()

    except:
        executor.shutdown()
        #sub.destroy_node()
        pub.destroy_node()
        rp.shutdown()

if __name__=="__main__":
    main()
