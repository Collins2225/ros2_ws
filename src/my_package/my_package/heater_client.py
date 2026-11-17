import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
from std_msgs.msg import Float32


class HeaterClient(Node):

    def __init__(self):
        super().__init__('heater_client')

        # Declare and read the two required parameters
        self.declare_parameter('low_temp', 18.0)
        self.declare_parameter('hight_temp', 23.0)

        self.low_temp = self.get_parameter('low_temp').value
        self.high_temp = self.get_parameter('hight_temp').value

        self.get_logger().info(
            f'Using thresholds: low_temp={self.low_temp}, hight_temp={self.high_temp}'
        )

        self.cli = self.create_client(SetBool, 'heater_control')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for heater_control service...')

        self.heater_state = False  # tracks last commanded state

        self.subscription = self.create_subscription(
            Float32,
            '/temp',
            self.temp_callback,
            10
        )

    def temp_callback(self, msg):
        temperature = msg.data
        self.get_logger().info(f'Current temperature: {temperature:.2f}')

        if temperature < self.low_temp and not self.heater_state:
            self.call_heater(True)
        elif temperature > self.high_temp and self.heater_state:
            self.call_heater(False)

    def call_heater(self, turn_on):
        req = SetBool.Request()
        req.data = turn_on
        future = self.cli.call_async(req)
        future.add_done_callback(self.response_callback)
        self.heater_state = turn_on

    def response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'Service response: {response.message}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = HeaterClient()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()