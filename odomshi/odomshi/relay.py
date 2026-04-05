import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry


class ImuOdomRelay(Node):

    def __init__(self):
        super().__init__('imu_odom_relay')

        # Subscribers
        self.imu_sub = self.create_subscription(
            Imu,
            '/imu',
            self.imu_callback,
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # Publishers
        self.imu_pub = self.create_publisher(Imu, '/imu_relayed', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom_relayed', 10)

        self.get_logger().info("IMU + Odom relay node started.")

        # --- Define covariances ---
        self.orientation_cov = [0.01, 0.0, 0.0,
                                 0.0, 0.01, 0.0,
                                 0.0, 0.0, 0.01]

        self.angular_velocity_cov = [0.02, 0.0, 0.0,
                                     0.0, 0.02, 0.0,
                                     0.0, 0.0, 0.02]

        self.linear_accel_cov = [0.04, 0.0, 0.0,
                                 0.0, 0.04, 0.0,
                                 0.0, 0.0, 0.04]

        self.pose_cov = [0.05, 0.0, 0.0, 0.0, 0.0, 0.0,
                         0.0, 0.05, 0.0, 0.0, 0.0, 0.0,
                         0.0, 0.0, 0.05, 0.0, 0.0, 0.0,
                         0.0, 0.0, 0.0, 0.01, 0.0, 0.0,
                         0.0, 0.0, 0.0, 0.0, 0.01, 0.0,
                         0.0, 0.0, 0.0, 0.0, 0.0, 0.01]

        self.twist_cov = [0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
                          0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
                          0.0, 0.0, 0.1, 0.0, 0.0, 0.0,
                          0.0, 0.0, 0.0, 0.02, 0.0, 0.0,
                          0.0, 0.0, 0.0, 0.0, 0.02, 0.0,
                          0.0, 0.0, 0.0, 0.0, 0.0, 0.02]

    # ---------------- IMU ----------------
    def imu_callback(self, msg: Imu):
        new_msg = Imu()

        new_msg.header.stamp = self.get_clock().now().to_msg()
        new_msg.header.frame_id = "imu_link"
        new_msg.orientation = msg.orientation
        new_msg.angular_velocity = msg.angular_velocity
        new_msg.linear_acceleration = msg.linear_acceleration

        # Inject covariances
        new_msg.orientation_covariance = self.orientation_cov
        new_msg.angular_velocity_covariance = self.angular_velocity_cov
        new_msg.linear_acceleration_covariance = self.linear_accel_cov

        self.imu_pub.publish(new_msg)

    # ---------------- ODOM ----------------
    def odom_callback(self, msg: Odometry):
        new_msg = Odometry()

        new_msg.header = msg.header
        new_msg.child_frame_id = msg.child_frame_id

        new_msg.pose = msg.pose
        new_msg.twist = msg.twist

        # Inject covariances
        new_msg.pose.covariance = self.pose_cov
        new_msg.twist.covariance = self.twist_cov

        self.odom_pub.publish(new_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ImuOdomRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()