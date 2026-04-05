#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Header
import random
import math

class ImuPublisher(Node):
    def __init__(self):
        super().__init__('imu_publisher')
        
        # Create a publisher for the /imu/data topic
        self.imu_pub = self.create_publisher(Imu, '/dummy_imu', 10)
        
        # Publish at 100 Hz (typical for IMU)
        timer_period = 0.01  # 100 Hz
        self.timer = self.create_timer(timer_period, self.publish_imu)
        
        self.get_logger().info('IMU publisher node started')
        
        # Simulated linear acceleration (m/s²)
        # When robot is stationary on flat ground, it should read gravity
        self.gravity = 9.81
        
    def publish_imu(self):
        imu = Imu()
        
        # Fill in the header
        imu.header.stamp = self.get_clock().now().to_msg()
        imu.header.frame_id = 'imu_link'
        
        # ============================================
        # ANGULAR VELOCITY (rad/s)
        # ============================================
        # Add Gaussian noise to angular velocity measurements
        noise_roll = random.gauss(0, 0.01)
        noise_pitch = random.gauss(0, 0.01)
        noise_yaw = random.gauss(0, 0.01)
        
        imu.angular_velocity.x = 0.0 + noise_roll      # Roll rate
        imu.angular_velocity.y = 0.0 + noise_pitch     # Pitch rate
        imu.angular_velocity.z = 0.0 + noise_yaw  # Yaw rate
        
        # Angular velocity covariance (3x3 matrix)
        imu.angular_velocity_covariance = [
            0.0001, 0.0, 0.0,
            0.0, 0.0001, 0.0,
            0.0, 0.0, 0.0001
        ]
        
        # ============================================
        # LINEAR ACCELERATION (m/s²)
        # ============================================
        # When stationary on flat ground, IMU measures gravity in z-axis
        # Add some noise and simulate small vibrations
        noise_ax = random.gauss(0, (0.05)**0.5)
        noise_ay = random.gauss(0, (0.05)**0.5)
        noise_az = random.gauss(0, (0.05)**0.5)
        
        imu.linear_acceleration.x = 0.0 + noise_ax
        imu.linear_acceleration.y = 0.0 + noise_ay
        imu.linear_acceleration.z = self.gravity + noise_az  # Gravity + noise

        sigma = 0.5
        
        # Linear acceleration covariance (3x3 matrix)
        imu.linear_acceleration_covariance = [
            sigma , 0.0, 0.0,
            0.0, sigma , 0.0,
            0.0, 0.0, sigma 
        ]
        
        # Publish the message
        self.imu_pub.publish(imu)

def main(args=None):
    rclpy.init(args=args)
    
    imu_publisher = ImuPublisher()
    
    try:
        rclpy.spin(imu_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        imu_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()