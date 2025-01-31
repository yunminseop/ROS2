import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from ultralytics import YOLO
import socket
import cv2
import numpy as np
import torch
from central_msg.msg import Slope
import gc

class UdpReceiverNode(Node):
    def __init__(self, model):
        super().__init__('udp_receiver_node')
        self.model = model

        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_socket.bind(("192.168.4.13", 8080))
        self.get_logger().info("Socket bound to 192.168.0.30:8080")

        self.publisher = self.create_publisher(Slope, "/slope", 10)
        self.execute_timer = self.create_timer(0.1, self.receive_and_command)

        self.unit = Conversion(1920, 1080, 16.1)

        self.ellipse_center = (320, 640)
        self.ellipse_axes = (320, 100)
        
        self.center = Centroid()
        self.slope = 0.0
        self.prev_masks = [0, 0, 640, 0, 640, 640, 0, 640]
        self.standard_point = (320, 540)
        self.intersection_finder = GetIntersection(self.ellipse_center, self.ellipse_axes)
        self.frame = None

    def receive_and_command(self):
        try:
            data, addr = self.udp_socket.recvfrom(65536)
            self.get_logger().info(f"Received data from {addr}")

            np_array = np.frombuffer(data, dtype=np.uint8)

            self.frame = cv2.imdecode(np_array, cv2.IMREAD_COLOR)

            self.frame = cv2.rotate(self.frame, cv2.ROTATE_180)
            self.frame = cv2.cvtColor(self.frame, cv2.COLOR_BGR2RGB)

            if self.frame is not None:
                frame_resized = cv2.resize(self.frame, (640, 640))
            
                results = self.model.predict(frame_resized)
                
                for result in results:
                    boxes = result.boxes.xyxy.cpu().numpy()
                    classes = result.boxes.cls.cpu().numpy()
                    """
                    cls_id = {0: center, 1: right, 2: left, 7: safety zone}
                    """
                    masks = result.masks.xy if result.masks else None
                    # print(classes)

                    if masks != None:
                        prev_masks = masks
                        for cls_id, mask in zip(classes, masks):
                            if cls_id == 0:
                                print(f"Class: {self.model.names[int(cls_id)]}")
                                self.center.get_centroid(mask)
                    elif prev_masks:
                                for cls_id, mask in zip(classes, prev_masks):
                                    if cls_id == 0:
                                        self.center.get_centroid(mask)

                dynamic_point = (self.center.centroid_x, self.center.centroid_y)
                self.intersection_finder.set_dynamic_line(dynamic_point)

                points = self.intersection_finder.calculate_intersection()
                
                if points:
                    valid_points = self.intersection_finder.filter_valid_points(points, 640, 640)
                    for point in valid_points:
                    
                        if (640-point[0])*(self.unit.p2cm()) < 320*self.unit.p2cm():
                            self.x = -(320 - point[0])*self.unit.p2cm()
                        else:
                            self.x = (point[0] - 320)*self.unit.p2cm()

                    self.y = (640-point[1])*self.unit.p2cm()*7.5

                self.slope = np.arctan(self.x/(0.396*self.y+6.3)) if point[1] != 0 else np.arctan(self.x)
                        
                self.slope = np.degrees(self.slope)

                msg = Slope()
                msg.slope = self.slope
                
                self.publisher.publish(msg)
                print(f"msg.slope: {msg.slope}")
                del self.frame
                gc.collect()

            else:
                self.get_logger().error("Failed to decode image")
        except Exception as e:
            self.get_logger().error(f"Error receiving or displaying image: {e}")

    def destroy_node(self):
        self.udp_socket.close()
        cv2.destroyAllWindows()
        self.get_logger().info("UDP Receiver Node stopped")
        super().destroy_node()


class Conversion:
    def __init__(self, w_res, h_res, inch):
        self.__w_res = w_res
        self.__h_res = h_res
        self.__inch = inch

        self.__PPI = np.sqrt(np.power(self.__w_res, 2)+np.power(self.__h_res, 2))/self.__inch

        self.x = 0
        self.y = 0

    def p2cm(self):
        return  2.54 / self.__PPI


class Centroid():
    def __init__(self):
        self.centroid_x, self.centroid_y = 0, 0

    def get_centroid(self, polygon):
        area = 0
        self.centroid_x = 0
        self.centroid_y = 0
        n = len(polygon)

        for i in range(n):
            j = (i + 1) % n
            factor = polygon[i][0] * polygon[j][1] - polygon[j][0] * polygon[i][1]
            area += factor
            self.centroid_x += (polygon[i][0] + polygon[j][0]) * factor
            self.centroid_y += (polygon[i][1] + polygon[j][1]) * factor
        area /= 2.0
        if area != 0:
            self.centroid_x /= (6 * area)
            self.centroid_y /= (6 * area)

class GetIntersection:
    def __init__(self, ellipse_center, ellipse_axes):
        self.h, self.k = ellipse_center  
        self.a, self.b = ellipse_axes   
        self.fixed_point = (320, 640) # center of a frame

    def set_dynamic_line(self, target_point):
        x0, y0 = self.fixed_point
        x1, y1 = target_point

        # slope
        self.m = (y1 - y0) / (x1 - x0) if x1 != x0 else float('inf')  # x1 == x0일 경우 수직선
        
        # y_intersection
        self.c = y0 - self.m * x0


    def calculate_intersection(self):
        """get an intersection"""

        if self.m == float('inf'):
            return[[320, 540], [320, 740]]
        
        A = (1 / self.a**2) + (self.m**2 / self.b**2)
        B = (2 * self.m * (self.c - self.k) / self.b**2) - (2 * self.h / self.a**2)
        C = ((self.h**2) / self.a**2) + ((self.c - self.k)**2 / self.b**2) - 1
        
        # D = b^2 - 4ac
        discriminant = B**2 - 4 * A * C

        if discriminant < 0:
            return None  # no intersection

        x1 = (-B + np.sqrt(discriminant)) / (2 * A)
        x2 = (-B - np.sqrt(discriminant)) / (2 * A)

        y1 = self.m * x1 + self.c
        y2 = self.m * x2 + self.c

        if x1 > 0 and y1 > 0 or x1 < 0 and y1 > 0 :
            self.upper_point = (x1, y1)
        elif x2 > 0 and y2 > 0 or x2 < 0 and y2 > 0:
            self.upper_point = (x2, y2)

        return [[x1, y1], [x2, y2]]

    def filter_valid_points(self, points, width, height):
        """filtering only valid point 1, 2 quadrant"""
        valid_points = [
            (int(x), int(y)) for x, y in points
            if 0 <= x <= width and 0 <= y <= height
        ]
        # print(f"valid_p: {valid_points}")
        return valid_points
    

def main(args=None):
    checkpoint_path = '/root/asap/data/best.pt'

    device = "cuda" if torch.cuda.is_available() else "cpu"
    print(f"Using device: {device}")

    model = YOLO(checkpoint_path, verbose=False).to(device)

    rclpy.init(args=args)
    node = UdpReceiverNode(model)

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()