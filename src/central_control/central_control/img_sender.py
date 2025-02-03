import rclpy
from rclpy.node import Node
import socket
import numpy as np
import cv2
from picamera2 import Picamera2
import time
import struct

class UdpSenderNode(Node):
    def __init__(self):
        super().__init__('udp_sender_node')

        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.target_address = ("192.168.100.8", 8080)

        self.picam2 = Picamera2()
        self.picam2.start()
        self.frame_count = 0

        self.timer = self.create_timer(0.033, self.send_udp)
        print(f"target_address:{self.target_address}")
        
    def send_udp(self):
        frame = self.picam2.capture_array()

        if frame is not None:
            frame_resized = cv2.resize(frame, (640, 640))
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 83]
            _, buffer = cv2.imencode('.jpg', frame_resized, encode_param)
            byte_data = buffer.tobytes()
            
            self.udp_socket.sendto(byte_data, self.target_address)
            self.get_logger().info(f"Sent image of size: {len(byte_data)} bytes")

    def destroy_node(self):
        self.picam2.close()
        self.udp_socket.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UdpSenderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
