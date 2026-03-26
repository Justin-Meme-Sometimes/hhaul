#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import socket
import json
import threading
import subprocess
import time

class DetectionBridgeNode(Node):
    def __init__(self):
        super().__init__('detection_bridge_node')
        self.pub = self.create_publisher(String, 'person_detections', 10)
        self.host = '0.0.0.0'
        self.port = 5555
        self.last_alarm_time = 0
        self.alarm_cooldown = 5.0  # seconds between alarms
        self.get_logger().info(f"Listening for Kria detections on port {self.port}...")
        self.listener_thread = threading.Thread(target=self.listen, daemon=True)
        self.listener_thread.start()

    def trigger_alarm(self):
        now = time.time()
        if now - self.last_alarm_time > self.alarm_cooldown:
            self.last_alarm_time = now
            subprocess.Popen(['espeak', 'Warning, person detected on ground'])
            self.get_logger().warn("ALARM TRIGGERED: Person laying down detected!")

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
                            detections = parsed.get("detections", [])
                            num = len(detections)
                            if num > 0:
                                self.get_logger().info(f"Published {num} person detection(s)")
                                for d in detections:
                                    aspect_ratio = d.get("aspect_ratio", "N/A")
                                    laying_down = d.get("laying_down", False)
                                    confidence = d.get("confidence", 0)
                                    status = "LAYING DOWN" if laying_down else "upright"
                                    self.get_logger().info(
                                        f"  -> confidence={confidence} "
                                        f"aspect_ratio={aspect_ratio} "
                                        f"posture={status}"
                                    )
                                    if laying_down:
                                        self.trigger_alarm()
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
