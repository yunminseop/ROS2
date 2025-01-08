from control_msgs.srv import MultiSpawn
from turtlesim.srv import TeleportAbsolute
import rclpy as rp
from rclpy.node import Node
import numpy as np
import time

class MultiSpawning(Node):
    def __init__(self):
        super().__init__("service_provider")
        self.service_provider = self.create_service(MultiSpawn, "turtle_spawned", self.callback_service)
        self.teleport = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')
        self.req_teleport = TeleportAbsolute.Request()

    def callback_service(self, request, response):

        location = float(request.num)

        prev_loc = location
        while 1:

            if prev_loc >= 11.00:
                break
            
            start = prev_loc
            
            self.req_teleport.x = start+0.001
            self.req_teleport.y = start+0.001
            self.req_teleport.theta = start+0.001
            
            response.x = [self.req_teleport.x]
            response.y = [self.req_teleport.y]
            response.theta = [self.req_teleport.theta]

            self.teleport.call_async(self.req_teleport)

            prev_loc = start + 0.1
            time.sleep(0.1)
            

        return response
    
def main(args=None):
    rp.init(args=args)
    new_service = MultiSpawning()

    rp.spin(new_service)
    rp.shutdown()

if __name__ == "__main__":
    main()