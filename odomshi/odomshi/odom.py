#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from std_msgs.msg import Header
import random

delta_t = 1/20
velocity = 0.5

class OdomPublisher(Node):
    def __init__(self):
        super().__init__('odom_publisher')
        
        # Create a publisher for the /odom topic
        self.odom_pub = self.create_publisher(Odometry, '/dummy_odom', 10)
        
        # Create a timer to publish at 20 Hz
        timer_period = delta_t  # seconds (20 Hz)
        self.timer = self.create_timer(timer_period, self.publish_odometry)
        
        self.get_logger().info('Odometry publisher node started')
        self.x = 0.0
        self.y = 0.0
        self.vx = velocity  # Set initial velocity
        self.vy = 0.0  
    
    def publish_odometry(self):
        # Add Gaussian noise to the position updates
        # The noise is ADDED to the current position, not replacing it
        noise_x = random.gauss(0, (0.05)**0.5)
        noise_y = random.gauss(0, (0.05)**0.5)
        
        # Update position with constant velocity + noise
        new_x = self.x + velocity * delta_t + noise_x
        new_y = self.y + noise_y  # FIXED: was "random.gauss(0, ...)" which resets to 0
        
        # Calculate velocities based on position change
        self.vx = (new_x - self.x) / delta_t 
        self.vy = (new_y - self.y) / delta_t 
        
        # Update positions
        self.x = new_x
        self.y = new_y
        
        odom = Odometry()
        
        # Fill in the header
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        
        # Set the child frame
        odom.child_frame_id = 'base_link'
        
        # Fill in the pose (position and orientation)
        odom.pose.pose.position.x = self.x 
        odom.pose.pose.position.y = self.y 
        odom.pose.pose.position.z = 0.0
        
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = 0.0
        odom.pose.pose.orientation.w = 1.0

        sigma = 0.05
        
        # Set pose covariance (6x6 matrix, row-major)
        odom.pose.covariance = [
            sigma , 0.0, 0.0, 0.0, 0.0, 0.0,  # x variance
            0.0, sigma , 0.0, 0.0, 0.0, 0.0,  # y variance
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,   # z (unused)
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,   # roll (unused)
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,   # pitch (unused)
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0    # yaw (unused)
        ]
        
        # Fill in the twist (linear and angular velocities)
        odom.twist.twist.linear.x = self.vx 
        odom.twist.twist.linear.y = self.vy 
        odom.twist.twist.linear.z = 0.0
        
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = 0.0 
        
        # Set twist covariance
        odom.twist.covariance = [
            2*sigma, 0.0, 0.0, 0.0, 0.0, 0.0,   # vx variance
            0.0, 2*sigma, 0.0, 0.0, 0.0, 0.0,   # vy variance
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,   # vz (unused)
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,   # vroll (unused)
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,   # vpitch (unused)
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0    # vyaw (unused)
        ]
        
        # Publish the message
        self.odom_pub.publish(odom)
        # Reduce logging frequency to avoid spam
        if int(self.x * 10) % 10 == 0:  # Log every ~2 seconds
            self.get_logger().info(f'Position: x={self.x:.2f}, y={self.y:.2f}')

def main(args=None):
    rclpy.init(args=args)
    
    odom_publisher = OdomPublisher()
    
    try:
        rclpy.spin(odom_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        odom_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()