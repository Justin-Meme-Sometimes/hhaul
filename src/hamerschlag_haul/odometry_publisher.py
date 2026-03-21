#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros
import math
import sys
import time

try:
    import qwiic_otos
except ImportError:
    print("ERROR: qwiic_otos not installed!")
    print("Run: pip3 install sparkfun-qwiic-otos --break-system-packages")
    exit(1)


class OdometryPublisher(Node):
    """
    Publishes odometry from SparkFun Qwiic OTOS sensor.
    Provides odom → base_link transform for SLAM and Nav2.
    """
    
    def __init__(self):
        super().__init__('odometry_publisher')
        
        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # Initialize OTOS sensor
        self.myOtos = qwiic_otos.QwiicOTOS()
        
        if not self.myOtos.is_connected():
            self.get_logger().error('OTOS sensor not found on I2C bus!')
            self.get_logger().error('Check wiring and I2C configuration.')
            return
        
        self.get_logger().info('OTOS sensor found, initializing...')
        
        # Initialize the device
        self.myOtos.begin()
        
        # Calibrate IMU
        self.get_logger().info('Ensure OTOS is flat and stationary!')
        self.get_logger().info('Calibrating IMU in 3 seconds...')
        time.sleep(3)
        
        self.myOtos.calibrateImu()
        self.get_logger().info('IMU calibrated!')
        
        # Reset tracking to origin
        self.myOtos.resetTracking()
        self.get_logger().info('OTOS odometry publisher ready')
        
        # Timer: 20 Hz update rate
        self.timer = self.create_timer(0.05, self.update_odometry)
        
        # Store previous position for velocity calculation
        self.prev_x = 0.0
        self.prev_y = 0.0
        self.prev_h = 0.0
        self.last_time = self.get_clock().now()
    
    def update_odometry(self):
        """Read OTOS sensor and publish odometry"""
        
        try:
            # Get position from OTOS (returns inches and degrees)
            myPosition = self.myOtos.getPosition()
            
            # Convert to SI units
            x_meters = myPosition.x * 0.0254      # inches to meters
            y_meters = myPosition.y * 0.0254      # inches to meters
            h_radians = myPosition.h * math.pi / 180.0  # degrees to radians
            
            # Create odometry message
            current_time = self.get_clock().now()
            
            odom = Odometry()
            odom.header.stamp = current_time.to_msg()
            odom.header.frame_id = 'odom'
            odom.child_frame_id = 'base_link'
            
            # Position
            odom.pose.pose.position.x = x_meters
            odom.pose.pose.position.y = y_meters
            odom.pose.pose.position.z = 0.0
            
            # Orientation (quaternion from heading)
            odom.pose.pose.orientation.x = 0.0
            odom.pose.pose.orientation.y = 0.0
            odom.pose.pose.orientation.z = math.sin(h_radians / 2.0)
            odom.pose.pose.orientation.w = math.cos(h_radians / 2.0)
            
            # Calculate velocity
            dt = (current_time - self.last_time).nanoseconds / 1e9
            if dt > 0:
                vx = (x_meters - self.prev_x) / dt
                vy = (y_meters - self.prev_y) / dt
                vth = (h_radians - self.prev_h) / dt
                
                odom.twist.twist.linear.x = vx
                odom.twist.twist.linear.y = vy
                odom.twist.twist.angular.z = vth
            
            # Covariance (OTOS is quite accurate)
            odom.pose.covariance[0] = 0.005   # x variance (0.5 cm)
            odom.pose.covariance[7] = 0.005   # y variance (0.5 cm)
            odom.pose.covariance[35] = 0.01   # theta variance (~0.6 degrees)
            
            # Publish odometry
            self.odom_pub.publish(odom)
            
            # Broadcast transform: odom → base_link
            t = TransformStamped()
            t.header.stamp = current_time.to_msg()
            t.header.frame_id = 'odom'
            t.child_frame_id = 'base_link'
            
            t.transform.translation.x = x_meters
            t.transform.translation.y = y_meters
            t.transform.translation.z = 0.0
            
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = math.sin(h_radians / 2.0)
            t.transform.rotation.w = math.cos(h_radians / 2.0)
            
            self.tf_broadcaster.sendTransform(t)
            
            # Update previous values
            self.prev_x = x_meters
            self.prev_y = y_meters
            self.prev_h = h_radians
            self.last_time = current_time
            
        except Exception as e:
            self.get_logger().error(f'Odometry update error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = OdometryPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
