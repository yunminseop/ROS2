import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from central_msg.msg import Slope
from central_msg.srv import Choosepath, Obstacle, Redlight
from std_msgs.msg import String
from std_msgs.msg import Int16
import random

class PIDController(Node):
    def __init__(self):
        super().__init__('pid_controller')
        
        self.cross_road_cnt = 0
    
        #종방향 파라미터
        self.__fix__ = 0.27
        self.absolute_linear_x = self.__fix__ # (절대 속도)
        self._fix_linear_x = self.__fix__
        self.linear_Kp = 0.1

        #횡방향 파라미터
        self.declare_parameter('Kp', 1.5)
        self.declare_parameter('Ki', 0.07)
        self.declare_parameter('Kd', 0.07)

        self.Kp = self.get_parameter('Kp').value
        self.Ki = self.get_parameter('Ki').value
        self.Kd = self.get_parameter('Kd').value

        self.prev_output = 0.0
        self.integral_limit = 10.0
        self.integral_lower_limit = -10.0
        
        self.pid_output = 0.0


        self.vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sign_publisher = self.create_publisher(String, '/sign', 10)
        
        self.create_subscription(Slope, '/slope', self.slope_callback, 10)
        self.create_subscription(String, '/signal', self.detection_callback, 10)
        
        
        self.manual_subscriber = self.create_subscription(Int16, "/manual_hint", self.manual_sign, 10)
        self.arrival_subscriber = self.create_subscription(Int16, "/arrival_hint", self.arrival_sign, 10)

        self.create_timer(0.03, self.linear_controller)

        # self.client_obstacle = self.create_client(Obstacle, "/pinky1/obstacle")
        # self.client_redlight = self.create_client(Redlight, "/pinky1/redlight")

        # self.create_timer(0.03, self.publish_cmd_vel)

        self.is_arrival = True

        self.prev_linear_x_error = 0.0
        self.linear_x_integral = 0.0
        self.target_linear_x = self.__fix__

        self.prev_error = 0.0
        self.integral = 0.0
        self.current_slope = 0.0
        
        self.target_slope = 0.0
        self.error = 0.0
        self.prev_pid_output = 0.0
        self.is_manual = True

    def arrival_sign(self, msg): # 목적지 도착 신호
        sign = msg.data
        if sign == 0:
            self.is_arrival = True
        elif sign == 1:
            self.is_arrival = False
        

    def manual_sign(self, msg): # 수동/자율 모드
        sign = msg.data
        if sign == 3:
            self.is_manual = True
            # self.get_logger().info(f"manual driving")
        elif sign == 4:
            self.is_manual = False
            # self.get_logger().info(f"autonomous driving")


    # def send_obstacle_request(self, is_obstacle):
    #     req = Obstacle.Request()
    #     req.is_obstacle = is_obstacle
    #     future = self.client_obstacle.call_async(req)
    #     return future

    # def send_redlight_request(self, is_red):
    #     req = Redlight.Request()
    #     req.is_red = is_red
    #     future = self.client_redlight.call_async(req)
    #     return future

    def detection_callback(self, msg): # 장애물 탐지에 따른 종방향 타겟속도 설정
        # self.get_logger().info(f'msg.data: {msg.data}')
        match msg.data:
            case ("green light" | "fine"):
                self.target_linear_x = self.__fix__
            case "reaccelerate":
                self.target_linear_x = self.__fix__
            case "100KM":
                self.target_linear_x = self.__fix__
            case "crossing":
                self.target_linear_x = 0.23
            case "child protect":
                self.target_linear_x = 0.13
            case ("human" | "goat" | "red light"):
                self.target_linear_x = 0.0


        # match msg.data:
        #     case "go":
        #         pass
        #     case "no right":
        #         pass
        #     case "right":
        #         pass
        #     case "left":
        #         pass
        
        # match msg.data:
        #     case "pinky":
        #         pass
        #     case "cross road":
        #         pass


    def linear_controller(self): # 종방향 속도 제어 (P 제어)
        error = self.target_linear_x - self._fix_linear_x

        # self.get_logger().info(f'curr_speed: {self._fix_linear_x}, target_speed: {self.target_linear_x}')
            
        if self.target_linear_x > self._fix_linear_x:
            self._fix_linear_x += self.linear_Kp * (error*1.5)
        else:
            self._fix_linear_x -= self.linear_Kp * (abs(error)*1.5)

        if 0.0 <= self._fix_linear_x <= 0.005:
            self._fix_linear_x = 0.0
            
        if self._fix_linear_x > self.__fix__:
            self._fix_linear_x = self.__fix__

            
   
    def slope_callback(self, msg): # 횡방향 속도 제어 (PID 제어)
        if not self.is_manual: # 자율주행 모드

            if not self.is_arrival: # 목적지에 도착하지 않았으면
                
                if self.target_linear_x <= 0.17:
                    self.Kp = 1.2
                else:
                    self.Kp = self.get_parameter('Kp').value

                error = msg.target_slope - msg.curr_slope
                self.integral += error
                derivative = error - self.prev_error
                
                if abs(error) < 0.01: 
                    self.integral = 0.0 

                if self.integral > self.integral_limit:
                    self.integral = self.integral_limit
                elif self.integral < self.integral_lower_limit:
                    self.integral = self.integral_lower_limit

                self.pid_output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
                self.prev_error = error
                
                self.pid_output /= -45.0

                if self.pid_output >= 0.99:
                    self.pid_output = 0.99
                elif self.pid_output <= -0.99:
                    self.pid_output = -0.99

                if self._fix_linear_x <= 0.02:
                    self.pid_output = 0.0
                    
                # if abs(self.pid_output - self.prev_pid_output) > 0.5:
                #     self.get_logger().info("remove outlier PID Output")
                #     self.pid_output = self.prev_pid_output


            # def publish_cmd_vel(self): # 최종 cmd_vel 발행

                # cmd_vel 토픽 설정
                cmd_vel = Twist()

                cmd_vel.linear.x = self._fix_linear_x #self.absolute_linear_x #
                
                # 종방향 속도가 너무 느리면 각속도 = 0
                if cmd_vel.linear.x <= 0.1:
                    self.pid_output = 0.0
                else:
                    cmd_vel.angular.z = self.pid_output

                self.prev_pid_output = self.pid_output
                # 목적지 도착 신호를 받으면 무조건 정차
                    
                self.vel_publisher.publish(cmd_vel)

            else: # 목적지에 도착했으면
                cmd_vel = Twist()

                self.get_logger().info(f"self.is_arrival: {self.is_arrival}, 정차 중..")
                # self.target_linear_x = 0.
                cmd_vel.linear.x = 0.
                cmd_vel.angular.z = 0.

                self.vel_publisher.publish(cmd_vel)
                self.is_manual = True

        else: # 수동주행 모드

            """ 여기서 self.is_arrival = False를 안 하는 이유:

                수동주행 도중 목적지 설정 없이 자율주행으로 전환할 때
                바로 자율주행을 시작하는 것을 막기 위해서 self.is_arrival = False를 하지 않는 것임.
                
                이 상태에서 자율주행을 시작하려면 사용자가 자율주행 모드로 변경 후
                목적지를 새로 지정하여 self.is_arrival을 False로 만들어야 함. 
                
                수동주행 모드에서는 self.is_arrival은 항상 True를 가짐. 이동하는 곳이 모두 목적지이기 때문."""
            return


def main(args=None):
    rclpy.init(args=args)
    node = PIDController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
