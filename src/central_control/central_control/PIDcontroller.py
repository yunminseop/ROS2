import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from central_msg.msg import Slope

class PIDController(Node):
    def __init__(self):
        super().__init__('pid_controller')

        self.declare_parameter('Kp', 1.5)
        self.declare_parameter('Ki', 0.1)
        self.declare_parameter('Kd', 0.08)

        self.Kp = self.get_parameter('Kp').value
        self.Ki = self.get_parameter('Ki').value
        self.Kd = self.get_parameter('Kd').value

        self.integral_limit = 10.0
        self.integral_lower_limit = -10.0 

        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.create_subscription(Slope, '/slope', self.slope_callback, 10)

        self.prev_error = 0.0
        self.integral = 0.0
        self.current_slope = 0.0

        self.target_slope = 0.0
        self.error = 0.0

        self.linear_x = 0.29

    def slope_callback(self, msg):
 
        error = msg.target_slope - msg.curr_slope
        self.integral += error
        derivative = error - self.prev_error
        
        if abs(error) < 0.01:
            self.integral = 0.0

        if self.integral > self.integral_limit:
            self.integral = self.integral_limit
        elif self.integral < self.integral_lower_limit:
            self.integral = self.integral_lower_limit

        pid_output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.prev_error = error
        
        pid_output /= -45.0

        if pid_output >= 0.99:
            pid_output = 0.99
        elif pid_output <= -0.99:
            pid_output = -0.99

        
        # 결과 출력
        self.get_logger().info(f'Target Slope: {msg.target_slope}, Current Slope: {msg.curr_slope}, Error: {error}, PID Output: {pid_output}')
        
        # Twist 메시지 발행
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.29  # 고정된 속도
        cmd_vel.angular.z = pid_output  # PID 제어에 따른 각속도
        
        self.publisher.publish(cmd_vel)

def main(args=None):
    rclpy.init(args=args)
    node = PIDController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
