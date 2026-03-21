#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import socket
import json
import threading

class DetectionBridgeNode(Node):
    def __init__(self):
        super().__init__('detection_bridge_node')
        self.pub = self.create_publisher(String, 'person_detections', 10)
        self.host = '0.0.0.0'
        self.port = 5555
        self.get_logger().info(f"Listening for Kria detections on port {self.port}...")
        self.listener_thread = threading.Thread(target=self.listen, daemon=True)
        self.listener_thread.start()

    def listen(self):
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind((self.host, self.port))
        server.listen(1)

        while True:
            self.get_logger().info("Waiting for Kria to connect...")
            conn, addr = server.accept()
            self.get_logger().info(f"Kria connected from {addr}")
            buf = ""

            try:
                while True:
                    data = conn.recv(4096).decode('utf-8')
                    if not data:
                        break
                    buf += data
                    while "\n" in buf:
                        line, buf = buf.split("\n", 1)
                        if line.strip():
                            msg = String()
                            msg.data = line.strip()
                            self.pub.publish(msg)
                            parsed = json.loads(line.strip())
                            num = len(parsed.get("detections", []))
                            if num > 0:
                                self.get_logger().info(f"Published {num} person detection(s)")
            except (ConnectionResetError, BrokenPipeError):
                self.get_logger().warn("Kria disconnected.")
            finally:
                conn.close()

def main(args=None):
    rclpy.init(args=args)
    node = DetectionBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
