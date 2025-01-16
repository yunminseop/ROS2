import rclpy as rp
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
import time

from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from control_msgs.action import DistTurtle
from central_control.individual import Individual

from rcl_interfaces.msg import SetParametersResult

import math

class TurtleSub_Action(Individual):
    def __init__(self, ac_server):
        super().__init__()
        self.ac_server = ac_server

    def callback(self, msg):
        self.ac_server.current_pose = msg
        # print("X:", msg.x, ", Y:", msg.y)

class DistTurtleServer(Node):
    def __init__(self):
        super().__init__("dist_turtle_action_server")
        self.total_dist = 0
        self.is_first_time = True
        self.current_pose = Pose()
        self.previous_pose = Pose()
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        # self.timer = self.create_timer(0.5, self.timer_callback)
        self.action_server = ActionServer(self, DistTurtle, 'dist_turtle', self.execute_callback)
        self.declare_parameter("quantile_time", 0.75)
        self.declare_parameter("almost_goal_time", 0.95)

        (quantile_Time, almost_time) = self.get_parameters(['quantile_time', 'almost_goal_time'])

        self.add_on_set_parameters_callback(self.parameter_callback)
    

    # def timer_callback(self):
    #     msg = Twist()
    #     msg.linear.x = 1.0
    #     msg.linear.y = 1.0
    #     msg.angular.z = 1.0

    #     self.publisher.publish(msg)

    def parameter_callback(self,params):
        for param in params:
            print(param.name, " is changed to ", param.value)

        return SetParametersResult(successful=True)
    
    def calc_diff_pose(self):
        if self.is_first_time:
            self.previous_pose.x = self.current_pose.x
            self.previous_pose.y = self.current_pose.y
            self.is_first_time = False

        diff_dist = math.sqrt((self.current_pose.x - self.previous_pose.x) ** 2 + (self.current_pose.y - self.previous_pose.y)**2)
        self.previous_pose = self.current_pose
        return diff_dist
    
    def execute_callback(self, goal_handle):
        feedback_msg = DistTurtle.Feedback()

        msg = Twist()
        msg.linear.x = goal_handle.request.linear_x
        msg.angular.z = goal_handle.request.angular_z

        while True:
            self.total_dist += self.calc_diff_pose()
            feedback_msg.remain_dist = goal_handle.request.dist - self.total_dist
            goal_handle.publish_feedback(feedback_msg)
            self.publisher.publish(msg)
            time.sleep(0.01)

            # if feedback_msg.remain_dist < 0.2 :
            #     break

        goal_handle.succeed()
        result = DistTurtle.Result()

        result.pos_x = self.current_pose.x
        result.pos_y = self.current_pose.y
        result.pos_theta = self.current_pose.theta
        result.result_dist = self.total_dist
        
        self.total_dist = 0
        self.is_first_time = True

        return result
    

def main(args=None):
    rp.init(args=args)
    executor = MultiThreadedExecutor()

    ac = DistTurtleServer()
    sub = TurtleSub_Action(ac_server= ac)

    executor.add_node(sub)
    executor.add_node(ac)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        sub.destroy_node()
        ac.destroy_node()
        rp.shutdown()
    

if __name__=="__main__":
    main()
