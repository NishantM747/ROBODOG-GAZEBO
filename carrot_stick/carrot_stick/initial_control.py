import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
import placo
import numpy as np
from placo_utils.visualization import robot_viz, robot_frame_viz, frame_viz, points_viz
from placo_utils.tf import tf

class Carrot_and_stick(Node):
    def __init__(self):  # ✅ Double underscores
        super().__init__('carrot_and_stick_controller')  # ✅ Double underscores, better name
        self.pub = self.create_publisher(String, '/dog_goal', 10)
        time_period = 100
        self.timer = self.create_timer(time_period, self.publish_controls)
        
        self.get_logger().info('carrots and sticks spawned')
    
    def publish_controls(self):
        command = String()
        command.data = "data: '1.0'"
        self.pub.publish(command)

def main(args=None):
    rclpy.init(args=args)
    
    carrot = Carrot_and_stick()
    
    try:
        rclpy.spin(carrot)
    except KeyboardInterrupt:
        pass
    finally:
        carrot.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()