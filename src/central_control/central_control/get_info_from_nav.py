import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from tf2_ros import TransformListener, Buffer
from std_msgs.msg import String
from std_msgs.msg import Int16
import numpy as np
import math

class WaypointFollowerNode(Node):
    def __init__(self):
        super().__init__('waypoint_follower_node')

        self.waypoint_sub = self.create_subscription(String, '/waypoint', self.waypoint_callback, 10)
        self.orientation_pub = self.create_publisher(Int16, '/orient_hint', 10)
        
        self.calculate = self.create_timer(0.03, self.get_arctan_difference)

        self.calculate_cnt = 0
        self.orient_list = []

        self.curr_x = None
        self.curr_y = None
        self.target_x = None
        self.target_y = None
        self.ori_cos = None
        self.ori_sin = None
        self.arrival_sign = None


    def waypoint_callback(self, msg):

        try:
            decoded_msg = eval(msg.data)
            self.curr_x = decoded_msg[0]
            self.curr_y = decoded_msg[1]
            self.target_x = decoded_msg[2]
            self.target_y = decoded_msg[3]
            self.ori_cos = decoded_msg[4]
            self.ori_sin = decoded_msg[5]
            self.arrival_sign = decoded_msg[6]
        except:
            self.get_logger().info(f"invalid msg: {msg.data}")
            return

        # self.get_logger().info(f"test: {self.ori_cos}, {type(self.ori_cos)}")

    def get_arctan_difference(self):
        try:

            curr_x, curr_y = self.curr_x, self.curr_y
            tar_x, tar_y = self.target_x, self.target_y
            
            x_diff = tar_x - curr_x
            y_diff = -(tar_y - curr_y)
            
            """내 위치에서 waypoint까지의 각도"""
            target_radian = math.atan2(y_diff, x_diff)
            target_degree = math.degrees(target_radian)

            converted_ori_cos_radian = math.cos(self.ori_cos)
            converted_ori_cos_degree = math.degrees(math.acos(converted_ori_cos_radian))
            converted_ori_cos_degree *= 2
            converted_ori_cos = (180 - converted_ori_cos_degree)% 360

            delta_degree = target_degree - converted_ori_cos

            if delta_degree > 0:
                clockwise = 360 - abs(delta_degree)
                revert_clockwise = 360 - clockwise
            elif delta_degree < 0:
                clockwise = abs(delta_degree)
                revert_clockwise = 360 - clockwise

            orient_msg = Int16()

            if clockwise < revert_clockwise:
                orient_msg.data = 1
            else:
                orient_msg.data = -1
           
            self.get_logger().info(f"orient_msg:{orient_msg.data}")
            self.orientation_pub.publish(orient_msg)
        
        except:
            self.get_logger().info("error")
            return

def main(args=None):
    rclpy.init(args=args)
    waypoint_follower_node = WaypointFollowerNode()
    rclpy.spin(waypoint_follower_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
