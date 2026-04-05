#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import serial
import math

class SerialBridgeNode(Node):

    def __init__(self):
        super().__init__('serial_bridge_node')

        # ----- ROS subscription -----
        self.subscription = self.create_subscription(
            JointState,
            '/joint_targets',
            self.joint_callback,
            10
        )

        # ----- Serial port -----
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200)

        port = self.get_parameter('port').value
        baud = self.get_parameter('baud').value

        self.ser = serial.Serial(port, baud, timeout=0.1)

        self.get_logger().info(f'Serial Bridge started on {port} @ {baud}')

    def joint_callback(self, msg):
        if not msg.position:
            return

        # # Convert radians → degrees
        # angles_deg = [math.degrees(a) for a in msg.position]

        # # Create a simple string packet (POC)
        # packet = ','.join(f'{a:.2f}' for a in angles_deg) + '\n'

        # received msg is in radians itself, no need of normalization
        angles_norm = [a for a in msg.position]

        packet = ','.join(f'{a:.3f}' for a in angles_norm) + '\n'


        self.ser.write(packet.encode('utf-8'))

        self.get_logger().info(f'Sent angles: {packet.strip()}')


def main(args=None):
    rclpy.init(args=args)
    node = SerialBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
