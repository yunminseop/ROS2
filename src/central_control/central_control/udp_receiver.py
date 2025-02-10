import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from central_msg.srv import Choosepath, Obstacle, Redlight
from std_msgs.msg import String
from ultralytics import YOLO
from std_msgs.msg import Float32MultiArray
from scipy.special import binom
from matplotlib.patches import Polygon
from std_msgs.msg import Int16
import time
import socket
import cv2
import numpy as np
import torch
from central_msg.msg import Slope
import struct
import math
import gc


class UdpReceiverNode(Node):
    def __init__(self, seg_model, det_model):
        super().__init__('udp_receiver_node')
        self.seg_model = seg_model
        self.det_model = det_model
        self.prev_redlight_status = False
        self.prev_obstacle_status = False

        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_socket.bind(("192.168.100.8", 8080))
        self.get_logger().info("Socket bound to 192.168.100.8:8080")

        # self.sign_subscriber = self.create_subscription(String, "/sign", self.sign_callback, 10)
        self.orient_subscriber = self.create_subscription(Int16, "/orient_hint", self.orient_determine, 10)

        self.publisher = self.create_publisher(Slope, "/slope", 10)
        self.signal_publisher = self.create_publisher(String, "/signal", 10)
        
        # self.avoid_timer = self.create_timer(0.033, self.avoid)
        self.execute_timer = self.create_timer(0.033, self.receive_and_command)

        self.srv_obstacle = self.create_service(Obstacle, "/pinky1/obstacle", self.obstacle_callback)
        self.srv_redlight = self.create_service(Redlight, "/pinky1/redlight", self.red_callback)
        
        self.__x, self.__y, self.__w, self.__h = 0, 400, 640, 240
        
        self.orient = None

        # 중앙선 검출 좌표 초기화
        self.central_x = 320
        self.central_y = 640

        self.prev_center = None
        self.unit = Conversion(1920, 1080, 16.1)
        self.center = Centroid()
        self.lookahead_distance = 60
        self.frame = None

        self.center_list = []

        self.choosepath = False
        self.obstacle = False   
        self.redlight = False
        self.avoid = False
        self.go_left = False
        self.no_right = False
        self.child_protect = False
        self.prev_child_protect = False
        self.destination = None
        self.center_objective = None

        self.car_position = np.array([320, 940])
        self.prev_slope = 0.0
        self.slow_cnt = 0

    def obstacle_callback(self,request, response):
        self.get_logger().info(f"obstacle: {request.is_obstacle}")
        response.avoid = self.obstacle
        return response

    def red_callback(self, request, response):
        self.get_logger().info(f"red: {request.is_red}")
        response.stop = self.redlight
        return response

    def orient_determine(self, msg):
        self.get_logger().info(f"msg.data: {msg.data}")
        self.orient = msg.data

    def choose_center(self, center_list):
        center = Centroid()
        centroid_list = []

        for idx, each in enumerate(center_list):
            center.get_centroid(each)
            centroid_list.append([idx, center.centroid_x])

        centroid_list.sort(key=lambda x: x[1])
        # self.get_logger().info(f"self.orient: {self.orient}")

        res = None
        match self.orient:
            case -1: # 좌회전
                self.get_logger().info(f"self.orient: {self.orient}")
                res = min(centroid_list, key=lambda x: x[1])
                self.get_logger().info(f"res: {res}")
                
            case 1: # 우회전
                res = max(centroid_list, key=lambda x: x[1])
        
        self.get_logger().info(f"최종 res: {res}")
        return res
    
        """ 계산된 무게중심의 x좌표중 가장 크거나(우회전) 작은 값(좌회전의 인덱스를 반환 """
            

    def receive_and_command(self):
        try:
            data, addr = self.udp_socket.recvfrom(65536)

            np_array = np.frombuffer(data, dtype=np.uint8)

            self.frame = cv2.imdecode(np_array, cv2.IMREAD_COLOR)

            self.frame = cv2.rotate(self.frame, cv2.ROTATE_180)
            self.frame = cv2.cvtColor(self.frame, cv2.COLOR_BGR2RGB)

            if self.frame is not None:
                frame_resized = cv2.resize(self.frame, (640, 640))

                self.ROI = frame_resized[self.__y:self.__y+self.__h, self.__x:self.__x+self.__w]

                # 중앙선 중심 좌표 추출 (ROI 크기의 hsv라는 이미지를 따로 만들어 추출만 진행)
                try:
                    hsv = cv2.cvtColor(self.ROI, cv2.COLOR_BGR2HSV)

                    lower_yellow = np.array([20, 100, 100])
                    upper_yellow = np.array([30, 255, 255])
                    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

                    kernel = np.ones((5, 5), np.uint8)
                    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
                    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

                    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

                    if contours:
                        largest_contour = max(contours, key=cv2.contourArea)  # 가장 큰 윤곽선 선택
                        M = cv2.moments(largest_contour)
                        
                        if M["m00"] != 0:  # 무게중심이 존재하는 경우
                            cx_roi = int(M["m10"] / M["m00"])  # ROI 내부 X 중심 좌표
                            cy_roi = int(M["m01"] / M["m00"])  # ROI 내부 Y 중심 좌표

                            # ROI 내부 좌표를 프레임 좌표로 변환
                            self.central_x = cx_roi + self.__x
                            self.central_y = cy_roi + self.__y
                except:
                    pass


                seg_results = self.seg_model.predict(frame_resized)
                det_results = self.det_model.predict(frame_resized)
                
                """ object detection process """
                
                if det_results:
                    if self.redlight == False and (self.obstacle == True or self.child_protect == True):
                        self.slow_cnt += 1

                    #  장애물 혹은 어린이보호구역이 아닌 상태로 200count가 지나면 재가속
                    if self.slow_cnt >= 200 and (self.obstacle == False and self.child_protect == False):
                        self.avoid = False
                        self.slow_cnt = 0
                        sign = String()
                        sign.data = "reaccelerate"
                        self.signal_publisher.publish(sign)

                    for det_result in det_results:
                
                        classes = det_result.boxes.cls.cpu().numpy()
                        boxes = det_result.boxes.xyxy.cpu().numpy()  # (x_min, y_min, x_max, y_max)
                        confidences = det_result.boxes.conf.cpu().numpy()

                        """ 우선 conf score 높은 장애물이 감지되면 obstacle 신호를 먼저 정하기.
                            1. 장애물 감지되면 obstacle = True
                            2. 장애물 안 보이면 obstacle = False
                                이건 이후 segmentation에서 장애물 회피동작에 영향을 줌."""

                        for cls_id, box, conf in zip(classes, boxes, confidences):
                            object_ = self.det_model.names[int(cls_id)]
                            # self.get_logger().info(f"object_: {object_}, conf: {conf}, box: {box}")
                            # sign = String()
                            # sign.data = object_

                            # # 적신호 
                            # if object_ == "red light" and 250 < box[3] < 300 and abs(box[3] - box[1]) > 150 and conf > 0.7:
                            #     # self.get_logger().info(f"red light size: ((b[0]: {box[0]}, b[1]: {box[1]}, b[2]: {box[2]}, b[3]: {box[3]})")
                            #     self.redlight = True

                            # # 교차로
                            # if object_ == "cross road" :#and 270 < box[2] - box[0] < 370 and 500 < box[3] < 600  and conf > 0.7:
                            #     self.get_logger().info(f"cross road size: ((b[0]: {box[0]}, b[1]: {box[1]}, b[2]: {box[2]}, b[3]: {box[3]})")
                            #     if self.redlight:
                            #         if self.prev_redlight_status == False:
                            #             self.signal_publisher.publish(sign)
                            #         self.prev_redlight_status = True
                            #     else:
                            #         """여기다가 내비게이션 interaction 코드 추가"""

                            # # 횡단보도
                            # if object_ == "crossing" and box[1] > 530 and conf > 0.8:
                            #     # self.get_logger().info(f"crossing size: ((b[0]: {box[0]}, b[1]: {box[1]}, b[2]: {box[2]}, b[3]: {box[3]})")
                            #     if self.redlight == True:
                            #         sign.data = "red light"
                            #         self.signal_publisher.publish(sign)
                            #     if self.redlight != True:
                            #         if self.prev_obstacle_status == False:
                            #             # self.obstacle = True
                            #             self.signal_publisher.publish(sign)
                            #             # self.prev_obstacle_status = True

                            # # 사람, 염소
                            # elif object_ in ["human", "goat"] and (380 < box[3] < 640) and (abs(box[3] - box[1]) > 200) and (290. < np.mean([box[2],box[0]]) < 370.) and conf > 0.8:
                            #     self.get_logger().info(f"self.object_: {object_}, {conf:.2f}, {box}, self.prev_obstacle_status: {self.prev_obstacle_status}")
                            #     if self.redlight != True:
                            #         self.obstacle = True
                            #         if self.prev_obstacle_status == False:
                            #             # if object_ == "human":
                            #                 # self.get_logger().info(f"human size: (b[0]: {box[0]}, b[1]: {box[1]}, b[2]: {box[2]}, b[3]: {box[3]})")
                            #             # if object_ == "goat":
                            #                 # self.get_logger().info(f"goat size: ((b[0]: {box[0]}, b[1]: {box[1]}, b[2]: {box[2]}, b[3]: {box[3]})")
                                    
                            #             self.signal_publisher.publish(sign)
                            #             self.prev_obstacle_status = True
                            
                            # # 장애물
                            # elif object_ == "obstacle" and (340 < box[3] < 640) and (abs(box[3] - box[1]) > 150) and (290. < np.mean([box[2],box[0]]) < 370.) and conf > 0.3:
                            #     self.get_logger().info(f"self.object_: {object_}, {conf:.2f}, {box}, self.prev_obstacle_status: {self.prev_obstacle_status}")
                            #     if self.redlight != True:
                            #         self.obstacle = True
                            #         if self.prev_obstacle_status == False:
                            #             # self.get_logger().info(f"car size: ((b[0]: {box[0]}, b[1]: {box[1]}, b[2]: {box[2]}, b[3]: {box[3]})")
                            #             self.signal_publisher.publish(sign)
                            #             self.prev_obstacle_status = True
                                            
                            # # 어린이 보호구역
                            # elif object_ == "child protect" and 250 < box[3] < 300 and abs(box[3]-box[1]) > 60 and conf > 0.8:
                            #     # self.get_logger().info(f"child protect size: ((b[0]: {box[0]}, b[1]: {box[1]}, b[2]: {box[2]}, b[3]: {box[3]})")
                            #     if self.redlight != True:
                            #         self.signal_publisher.publish(sign)
                            #         if self.prev_child_protect == False:
                            #             self.child_protect = True
                            #             self.prev_child_protect = True

                            # # 100 km 표지판
                            # elif object_ == "100KM" and conf > 0.8:
                            #     # self.get_logger().info(f"100KM size: ((b[0]: {box[0]}, b[1]: {box[1]}, b[2]: {box[2]}, b[3]: {box[3]})")
                            #     if self.redlight != True and self.child_protect:
                            #         self.prev_child_protect = False
                            #         self.child_protect = False
                            #         self.signal_publisher.publish(sign)

                            # # 청신호
                            # elif object_ == "green light" and conf > 0.8:
                            #     self.redlight = False
                            #     # self.get_logger().info(f"green light size: ({box}, self.redlight: {self.redlight}, self.obstacle: {self.obstacle}")
                            #     if self.obstacle != True:
                            #         self.signal_publisher.publish(sign)
                            #         self.prev_redlight_status = False
            
                            # # 우회전 금지 화살표
                            # elif object_ == "no right" and 300 < np.mean([box[0], box[2]]) < 340 and conf > 0.8:
                            #     self.no_right = True
                            #     self.prev_no_right = True
                
                """ segmentation process """
                for seg_result in seg_results:
                    
                    classes = seg_result.boxes.cls.cpu().numpy() #cls_id = {0: center, 1: right, 2: left, 7: safety zone}
                    masks = seg_result.masks.xy if seg_result.masks else None
                    confidences = seg_result.boxes.conf.cpu().numpy()
                    

                    # masks 배열을 탐방하며 하나하나 분석
                    if masks is not None and len(masks) > 0:
                        self.center_list = []
                        for cls_id, mask, conf in zip(classes, masks, confidences):
                            if cls_id == 0: # center가 존재 시 center_list에 추가
                                # self.get_logger().info("1. center 존재")
                                self.center_list.append(mask)
                                # 만약 center가 없으면 center_list는 빈 배열이 됨.
                                
                    try:
                        # 만약 prev_center가 존재하고 center_list가 빈 배열이면 prev_center을 center로 취급
                        if len(self.prev_center) > 0 and len(self.center_list) == 0:
                            # self.get_logger().info("2. center 미존재, prev_center 존재 확인")
                            self.center_list = []
                            self.center_list.append(self.prev_center)

                        # prev_center, center 둘 다 없으면 그냥 정면을 center로 취급
                        elif len(self.prev_center) == 0 and len(self.center_list) == 0:
                            # self.get_logger().info("3. NO mask, prev_center")
                            self.center_list = []
                            self.center_list.append(np.array([320, 640]))
                    except:
                        self.get_logger().info("error")
                        # center_list가 없는 예외사항이니 error 발생


                    # try:
                    #     self.get_logger().info(f"center_list 개수: {len(self.center_list)}개")
                    # except:
                    #     pass
                    #     self.get_logger().info(f"center_list has only one element.")


                    if len(self.center_list) > 1:
                        """ res는 최종 결정된 center의 인덱스 """
                        res = self.choose_center(self.center_list)
                        self.destination = self.center_list[res]
                    else:
                        self.destination = self.center_list[0]
                        # self.get_logger().info(f"destination shape: {self.center_list[0]}")
                    
                    
                    self.prev_center = self.destination
                    # self.get_logger().info(f"prev_center: {self.prev_center.shape}")

                
                    self.center.get_centroid(self.destination)


            # Pure pursuit 알고리즘으로 조향각 계산
            processor = PurePursuit(self.destination, self.lookahead_distance)
            
            lookahead_distance, self.bezier_points = processor.get_bezier_points(self.car_position, (self.center.centroid_x, self.center.centroid_y))

            bezier_path = processor.bezier_curve(self.bezier_points)
            lookahead_point = processor.find_lookahead_point(bezier_path, self.car_position, lookahead_distance)
            
            slope = self.get_slope(lookahead_point)


            # 이전 조향각과 계산된 조향각의 차이가 비정상적으로 클 경우 무시
            try:
                if math.copysign(1,slope) != math.copysign(1,self.prev_slope) and np.abs(slope - self.prev_slope) > 10:
                    # self.get_logger().info(f"보정 전 slope: {slope:.3f} deg")
                    # self.get_logger().info(f"보정 후 slope: {self.prev_slope:.3f} deg")
                    slope = self.prev_slope
                else:
                    self.prev_slope = slope
            except:
                print("Prev_slope doesnt exist.")
            
            # 계산된 조향각을 토픽으로 발행
            msg = Slope()

            msg.curr_slope = slope
            msg.target_slope = 0.0
            msg.diff = 0.0 - slope

            # print(msg.curr_slope, msg.target_slope, msg.diff)
            # self.get_logger().info(f"Publishing: {msg.curr_slope}, {msg.target_slope}, {msg.diff}")

            self.publisher.publish(msg)

            
        except Exception as e:
            self.get_logger().error(f"Error receiving or displaying image: {e}")
        
    def get_slope(self, lookahead_point):
        x1, y1 = self.car_position
        x2, y2 = lookahead_point

        delta_y = y1 - y2
        delta_x = x1 - x2

        desired_angle = math.atan2(delta_x, delta_y)

        vehicle_angle = 0

        steering_angle = desired_angle - vehicle_angle

        steering_angle = math.degrees((steering_angle + math.pi) % (2 * math.pi) - math.pi)

        return steering_angle

    
    def destroy_node(self):
        self.udp_socket.close()
        cv2.destroyAllWindows()
        self.get_logger().info("UDP Receiver Node stopped")
        super().destroy_node()


class PurePursuit():
    def __init__(self, lane_polygon, lookahead_distance):
        self.lane_polygon = lane_polygon
        self.lookahead_distance = lookahead_distance

    def get_bezier_points(self, car_position, centroid):
        
        trans_polygon = self.lane_polygon.copy()
        dest = self.find_nearest_value(trans_polygon[:, 0], centroid[0])
        target_y = trans_polygon[trans_polygon[:, 0]==dest][0][1]

        dist = centroid[1] - target_y

        if dist > 0:
            trans_polygon[:, 1] += int(dist)

        else:
            trans_polygon[:, 1] -= int(dist)
        
        if 0 <= np.abs(dist) <= 100:
            """ dist가 100이면 lah_d+=50
                dist가 50이면 lah_d+=100
                즉, 무게중심과 edge간 거리가 가까워질수록 lah_d는 비례증가"""
            self.lookahead_distance += (100 - np.abs(dist))
        

        sort_index = np.argsort(trans_polygon[:, 1])
        y_max = trans_polygon[sort_index[0]]

        if dist < 100:
            """ car_position ~ centroid"""
            mid_control1 = (car_position[0]-(car_position[0] - centroid[0]) / 3, 1000 - ((car_position[1] - centroid[1]) * 5 / 10))
            mid_control2 = (car_position[0]-(car_position[0] - centroid[0]) * 2 / 3, 1000- ((car_position[1] - centroid[1]) * 8 / 10))

            """ centroid ~ y_max"""
            mid_control3 = (centroid[0]-(centroid[0] - y_max[0]) / 3, centroid[1] - ((centroid[1] - y_max[1]) * 5 / 10))
            mid_control4 = (centroid[0]-(centroid[0] - y_max[0]) * 2 / 3, centroid[1]- ((centroid[1] - y_max[1]) * 8 / 10))
            
            return (self.lookahead_distance, (car_position, mid_control1, mid_control2, mid_control3, mid_control4, y_max))
        else:
            """ car_position ~ centroid"""
            mid_control1 = (car_position[0]-(car_position[0] - centroid[0]) / 3, 1000 - ((car_position[1] - centroid[1]) * 5 / 10))
            mid_control2 = (car_position[0]-(car_position[0] - centroid[0]) * 2 / 3, 1000- ((car_position[1] - centroid[1]) * 8 / 10))
            return (self.lookahead_distance, (car_position, mid_control1, mid_control2, centroid)) 
            

    def bezier_curve(self, bezier_points, num_points=100):
        n = len(bezier_points) - 1
        t_values = np.linspace(0, 1, num_points)
        curve = np.zeros((num_points, 2))
    
        for i in range(n + 1):
            bernstein_poly = binom(n, i) * (t_values ** i) * ((1 - t_values) ** (n - i))
            curve += np.outer(bernstein_poly, bezier_points[i])
        
        return curve

    def find_lookahead_point(self, curve, current_pos, lookahead_distance):
        distances = np.linalg.norm(curve - current_pos, axis=1)
        idx = np.argmin(np.abs(distances - lookahead_distance))
        return curve[idx]

    def find_nearest_value(self, arr, value):
        idx = np.argmin(np.abs(arr - value))
        return arr[idx]


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



def main(args=None):
    seg_checkpoint_path = '/root/asap/data/seg_best5.pt'
    det_checkpoint_path = '/root/asap/data/det_best4.pt'

    device = "cuda" if torch.cuda.is_available() else "cpu"
    print(f"Using device: {device}")

    seg_model = YOLO(seg_checkpoint_path, verbose=False).to(device)
    det_model = YOLO(det_checkpoint_path, verbose=False).to(device)

    rclpy.init(args=args)
    node = UdpReceiverNode(seg_model, det_model)
    
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()