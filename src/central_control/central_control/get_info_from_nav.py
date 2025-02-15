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
        self.arrival_pub = self.create_publisher(Int16, '/arrival_hint', 10)
        self.manual_pub = self.create_publisher(Int16,'/manual_hint', 10)
        self.calculate = self.create_timer(0.03, self.get_arctan_difference)

        self.calculate_cnt = 0
        self.orient_list = []

        self.curr_x = None
        self.curr_y = None
        self.target_x = None
        self.target_y = None
        self.ori_cos = None
        self.ori_sin = None
        self.is_arrival = True
        self.prev_arrival = True
        self.is_manual = False


    def waypoint_callback(self, msg):
            # self.get_logger().info(f"msg: {msg}")
        try:
            decoded_msg = eval(msg.data)

            # self.get_logger().info(f"decoded_msg: {decoded_msg}, type: {type(decoded_msg)}")
            self.curr_x = decoded_msg[0]
            self.curr_y = decoded_msg[1]
            self.target_x = decoded_msg[2]
            self.target_y = decoded_msg[3]
            self.ori_cos = decoded_msg[4]
            self.ori_sin = decoded_msg[5]
            self.is_arrival = decoded_msg[6] # True: 0 False: 2
            self.is_manual = decoded_msg[7]  # True: 3 False: 4

            # self.get_logger().info(f"1... {decoded_msg[0]},{decoded_msg[1]},{decoded_msg[2]},{decoded_msg[3]},{decoded_msg[4]},{decoded_msg[5]},{decoded_msg[6]}")
            # self.get_logger().info(f"2... {self.curr_x},{self.curr_y},{self.target_x},{self.target_y},{self.ori_cos},{self.ori_sin},{self.is_arrival}")
            # self.get_logger().info(f"cos값: {self.ori_cos}, cos_deg: {math.degrees(math.acos(self.ori_cos))}")
            
        except:
            self.get_logger().info(f"invalid msg: {msg.data}")
            return


    def get_arctan_difference(self):
        
        # 도착 상태가 아니면
        if not self.is_arrival:
            # self.get_logger().info(f"is_arrival: {self.is_arrival}")
            try:

                # self.get_logger().info(f"여기까진 시도..")
                # self.get_logger().info(f"2... {self.curr_x},{self.curr_y},{self.target_x},{self.target_y},{self.ori_cos},{self.ori_sin},{self.is_arrival}")
                curr_x, curr_y = self.curr_x, self.curr_y
                tar_x, tar_y = self.target_x, self.target_y
                
                x_diff = tar_x - curr_x
                y_diff = -(tar_y - curr_y)
                
                
                """내 위치에서 waypoint까지의 각도"""
                target_radian = math.atan2(y_diff, x_diff)
                target_degree = (math.degrees(target_radian))%360
                
                # self.get_logger().info(f"target_r: {target_radian}, target_d: {target_degree}")
                # converted_ori_cos_radian = math.cos(self.ori_cos)

                converted_ori_cos_degree = math.degrees(math.acos(self.ori_cos))
                converted_ori_cos_degree *= 2
                converted_ori_cos = (180 - converted_ori_cos_degree)% 360
                
                delta_degree = target_degree - converted_ori_cos

                # self.get_logger().info(f"변환 후 각도: {converted_ori_cos}")

                # self.get_logger().info(f"wp까지의 각도: {target_degree:.1f}, 바라보는 각도: {converted_ori_cos:.1f}, diff: {delta_degree}")

                if delta_degree > 0:
                    clockwise = 360 - abs(delta_degree)
                    revert_clockwise = 360 - clockwise
                elif delta_degree < 0:
                    clockwise = abs(delta_degree)
                    revert_clockwise = 360 - clockwise

                orient_msg = Int16()
                arrival_msg = Int16()
                manual_msg = Int16()

                if clockwise < revert_clockwise and abs(clockwise) > 30.:
                    orient_msg.data = 1 # 교차로에서 우회전
                    self.get_logger().info(f"diff: {clockwise}, orient_msg:{orient_msg.data}, 우회전")
                elif clockwise < revert_clockwise and abs(clockwise) < 30.:
                    orient_msg.data = -1 # 교차로에서 직진
                    self.get_logger().info(f"diff: {clockwise}, orient_msg:{orient_msg.data}, 직진")

                elif revert_clockwise < clockwise and abs(revert_clockwise) > 30.:
                    orient_msg.data = -1 # 교차로에서 좌회전
                    self.get_logger().info(f"diff: {revert_clockwise}, orient_msg:{orient_msg.data}, 좌회전")
                elif revert_clockwise < clockwise and abs(revert_clockwise) < 30.:
                    orient_msg.data = -1 # 교차로에서 직진
                    self.get_logger().info(f"diff: {revert_clockwise}, orient_msg:{orient_msg.data}, 직진")


                # 수동 주행이면
                if self.is_manual:
                    manual_msg.data = 3 # 수동 
                else:
                    manual_msg.data = 4 # 자율
                
                self.manual_pub.publish(manual_msg)

                # 도착 상태에서 다시 출발하기위해 상태를 해제
                if self.prev_arrival == True:
                    
                    arrival_msg.data = 1 # 도착 해제
                    self.arrival_pub.publish(arrival_msg)
                    self.prev_arrival = False
            
                # self.get_logger().info(f"시계방향으로(우회전): {clockwise}")
                # self.get_logger().info(f"반시계방향으로(좌회전): {revert_clockwise}")
                
                self.orientation_pub.publish(orient_msg)
        
            except:
                pass
                # self.get_slogger().info("error")
                return
        else:

            self.get_logger().info("정차 명령 중...")

            arrival_msg = Int16()

            arrival_msg.data = 0

            self.arrival_pub.publish(arrival_msg)

            # 도착했으니 이전 도착 상태 True
            self.prev_arrival = True
            

def main(args=None):
    rclpy.init(args=args)
    waypoint_follower_node = WaypointFollowerNode()
    rclpy.spin(waypoint_follower_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
