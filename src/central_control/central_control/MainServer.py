import rclpy as rp
from rclpy.node import Node
import cv2
import numpy as np
from ultralytics import YOLO
import torch 
import torch.nn as nn
import logging
from geometry_msgs.msg import Twist

# YOLO verbose False 적용
logging.getLogger("ultralytics").setLevel(logging.ERROR)



class MainServer(Node):
    def __init__(self, video_path, model):
        super().__init__("MainServer")
        self.video_path = video_path
        self.model = model

        self.unit = Conversion(1920, 1080, 16.1)

        self.ellipse_center = (320, 640)  # 타원의 중심
        self.ellipse_axes = (320, 100)    # 타원의 반지름 (a, b)
        
        self.center = Centroid()
        self.slope = 0.0

        # self.cap = cv2.VideoCapture(self.video_path)

        self.standard_point = (320, 540)

        if not self.cap.isOpened():
            print("Error: Couldn't open the video file.")
            exit()

        # self.fps = self.cap.get(cv2.CAP_PROP_FPS)

        self.intersection_finder = GetIntersection(self.ellipse_center, self.ellipse_axes)


        self.commander = self.create_publisher(Twist, "/slope_vel", 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)

        
    def timer_callback(self):
            dynamic_point = (self.center.centroid_x, self.center.centroid_y)
            self.intersection_finder.set_dynamic_line(dynamic_point)

            ret, frame = self.cap.read()
            # frame = cv2.rotate(frame, cv2.ROTATE_180)
            
            if not ret:
                print("Error: Couldn't read a frame.")
                return
            
            frame_resized = cv2.resize(frame, (640, 640))
            
            results = self.model.predict(frame_resized)
            
            for result in results:
                boxes = result.boxes.xyxy.cpu().numpy()
                classes = result.boxes.cls.cpu().numpy()
                """
                cls_id = {0: center, 1: right, 2: left, 7: safety zone}
                """
                masks = result.masks.xy if result.masks else None

                if masks != None:
                    prev_masks = masks
                    for cls_id, mask in zip(classes, masks):
                        if cls_id == 0:
                            
                            self.center.get_centroid(mask)

                else:
                    for mask in prev_masks:
                        self.center.get_centroid(mask)


            annotated_frame = results[0].plot(boxes=False)

            # cv2.ellipse(annotated_frame, self.ellipse_center, self.ellipse_axes, 0, 0, 360, (150, 150, 150), 1)

            # cv2.circle(annotated_frame, self.standard_point, 5, (0, 0, 255), -1, cv2.LINE_AA)

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
            
            delay = int(1000 / self.fps) # original fps

            (x, z) = self.Calculate(self.slope)
            

            msg = Twist()
            msg.linear.x = x
            msg.angular.z = z
        
            self.commander.publish(msg)

            if cv2.waitKey(delay) & 0xFF == ord('q'):
                return
            
    def Calculate(self, slope):
        """
        -45 ~ +45
        """
        x = 0.29
        if -45.0 < slope < 0.0:
            z = 0.52

        elif 0.0< slope < 45.0:
            z = -0.52
            
        return (x, z)

                

        



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

    def set_dynamic_line(self, target_point):
        x0, y0 = self.fixed_point
        x1, y1 = target_point

        if y1 >= 640:
            y1 = 639
            
        # slope
        self.m = (y1 - y0) / (x1 - x0) if x1 != x0 else float('inf')  # x1 == x0일 경우 수직선
        
        # y_intersection
        self.c = y0 - self.m * x0

class VideoProcessor:
    def __init__(self, video_path, model):
        self.video_path = video_path
        self.model = model

        self.unit = Conversion(1920, 1080, 16.1)

        self.ellipse_center = (320, 640)  # 타원의 중심
        self.ellipse_axes = (320, 100)    # 타원의 반지름 (a, b)
        
        self.center = Centroid()
        self.slope = 0.0

    def process_video(self):
        cap = cv2.VideoCapture(self.video_path)
        standard_point = (320, 540)

        if not cap.isOpened():
            print("Error: Couldn't open the video file.")
            exit()

        fps = cap.get(cv2.CAP_PROP_FPS)

        intersection_finder = GetIntersection(self.ellipse_center, self.ellipse_axes)

        while True:

            dynamic_point = (self.center.centroid_x, self.center.centroid_y)
            intersection_finder.set_dynamic_line(dynamic_point)

            ret, frame = cap.read()
            frame = cv2.rotate(frame, cv2.ROTATE_180)
            
            if not ret:
                print("Error: Couldn't read a frame.")
                break
            
            frame_resized = cv2.resize(frame, (640, 640))
            
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
                            # print(f"Class: {model.names[int(cls_id)]}")
                            self.center.get_centroid(mask)

                else:
                    for mask in prev_masks:
                        self.center.get_centroid(mask)


            annotated_frame = results[0].plot(boxes=False)

            cv2.ellipse(annotated_frame, self.ellipse_center, self.ellipse_axes, 0, 0, 360, (150, 150, 150), 1)

            cv2.circle(annotated_frame, standard_point, 5, (0, 0, 255), -1, cv2.LINE_AA)

            points = intersection_finder.calculate_intersection()
            
            if points:
                valid_points = intersection_finder.filter_valid_points(points, 640, 640)
                for point in valid_points:
                    cv2.circle(annotated_frame, point, 5, (0, 255, 0), -1, cv2.LINE_AA)
                
                if (640-point[0])*(self.unit.p2cm()) < 320*self.unit.p2cm():
                    self.x = -(320 - point[0])*self.unit.p2cm()
                else:
                    self.x = (point[0] - 320)*self.unit.p2cm()

                self.y = (640-point[1])*self.unit.p2cm()*7.5

            self.slope = np.arctan(self.x/(0.396*self.y+6.3)) if point[1] != 0 else np.arctan(self.x)
                    
            self.slope = np.degrees(self.slope)
            
            cv2.imshow('center_pursuit', annotated_frame)
            
            delay = int(1000 / fps) # original fps
            
            if cv2.waitKey(delay) & 0xFF == ord('q'):
                break

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
    video_path = '/home/ms/Downloads/video_output5.mp4'
    checkpoint_path = '/home/ms/Downloads/best.pt'

    model = YOLO(checkpoint_path, verbose=False)

    processor = VideoProcessor(video_path, model)
    processor.process_video()

    rp.init(args = args)

    server = MainServer()
    rp.spin(server)

    server.destroy_node()
    rp.shutdown()


if __name__ == "__main__":
    main()