from control_msgs.srv import MultiSpawn
from turtlesim.srv import TeleportAbsolute
import rclpy as rp
from rclpy.node import Node
import numpy as np

class MultiSpawning(Node):
    def __init__(self):
        super().__init__("service_provider")
        self.service_provider = self.create_service(MultiSpawn, "new_turtle_spawned", self.callback_service)
        self.teleport = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')
        self.req_teleport = TeleportAbsolute.Request()
        # self.count = 0

    def callback_service(self, request, response):
        # self.count += 1
        
        self.req_teleport.x = float(request)
        
        self.teleport.call_async(self.req_teleport)
        print(self.req_teleport)
        return response
    
    # def draw_circle(self):
    #     result = 
    #     return result
    
def main(args=None):
    rp.init(args=args)
    new_service = MultiSpawning()

    rp.spin(new_service)
    rp.shutdown()

if __name__ == "__main__":
    main()