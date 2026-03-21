#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros
import qwiic_otos
import math
import sys

class OTOSOdomNode(Node):
    def __init__(self):
        super().__init__('otos_odom_node')

        # Match Cartographer's BEST_EFFORT QoS
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )

        self.odom_pub = self.create_publisher(Odometry, '/odom', qos)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Initialize OTOS sensor
        self.otos = qwiic_otos.QwiicOTOS()

        if not self.otos.is_connected():
            self.get_logger().error("OTOS not connected! Check wiring.")
            sys.exit(1)

        self.otos.begin()
        self.get_logger().info("Calibrating OTOS IMU, keep sensor flat and still...")
        self.otos.calibrateImu()
        self.otos.resetTracking()
        self.get_logger().info("OTOS ready!")

        # Publish at 50Hz
        

    def publish_odom(self):
        pos = self.otos.getPosition()

        # Convert inches to meters, degrees to radians
        x = pos.x * 0.0254
        y = pos.y * 0.0254
        heading_rad = math.radians(pos.h)

        # Quaternion from yaw only
        qz = math.sin(heading_rad / 2.0)
        qw = math.cos(heading_rad / 2.0)

        now = self.get_clock().now().to_msg()

        # Publish TF odom -> base_link
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(t)

        # Publish Odometry message
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        self.odom_pub.publish(odom)

def main():
    rclpy.init()
    node = OTOSOdomNode()
    while rclpy.ok():
            node.publish_odom()
            rclpy.spin_once(node, timeout_sec=0)
    rclpy.shutdown()

if __name__ == '__main__':
    main()  
