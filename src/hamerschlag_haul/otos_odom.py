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
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )
        self.odom_pub = self.create_publisher(Odometry, '/odom', qos)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.otos = qwiic_otos.QwiicOTOS()
        if not self.otos.is_connected():
            self.get_logger().error("OTOS not connected! Check wiring.")
            sys.exit(1)
        self.otos.begin()
        self.get_logger().info("Calibrating OTOS IMU, keep sensor flat and still...")
        self.otos.calibrateImu()
        self.otos.resetTracking()
        self.get_logger().info("OTOS ready!")

        self.timer = self.create_timer(0.02, self.publish_odom)  # 50Hz

    def publish_odom(self):
        pos = self.otos.getPosition()
        vel = self.otos.getVelocity()

        x = pos.x * 0.0254
        y = pos.y * 0.0254
        heading_rad = math.radians(pos.h)

        qz = math.sin(heading_rad / 2.0)
        qw = math.cos(heading_rad / 2.0)

        now = self.get_clock().now().to_msg()

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
        odom.twist.twist.linear.x = vel.x * 0.0254
        odom.twist.twist.linear.y = vel.y * 0.0254
        odom.twist.twist.angular.z = math.radians(vel.h)

        # Pose covariance (row-major 6x6): x, y, z, roll, pitch, yaw
        # Higher values = less trust in odometry, more reliance on scan matching
        # Note that this gets changed alot and we depend more on Lidar for this
        odom.pose.covariance[0] = 0.5    # x
        odom.pose.covariance[7] = 0.5    # y
        odom.pose.covariance[35] = 2.0   # yaw -- high: OTOS heading drifts, let LIDAR handle rotation

        # Twist covariance
        odom.twist.covariance[0] = 0.3   # vx
        odom.twist.covariance[7] = 0.3   # vy
        odom.twist.covariance[35] = 1.0  # vyaw

        self.odom_pub.publish(odom)

def main():
    rclpy.init()
    node = OTOSOdomNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
