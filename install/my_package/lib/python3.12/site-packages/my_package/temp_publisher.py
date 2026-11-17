import random

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class TempPublisher(Node):

    def __init__(self):
        super().__init__('temp_publisher')
        self.publisher_ = self.create_publisher(Float32, '/temp', 10)
        self.timer = self.create_timer(2.0, self.timer_callback)
        self.temp = 20.0

    def timer_callback(self):
        self.temp += random.uniform(-2.0, 2.0)
        msg = Float32()
        msg.data = self.temp
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published temp: {self.temp:.2f}')


def main(args=None):
    rclpy.init(args=args)
    node = TempPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()