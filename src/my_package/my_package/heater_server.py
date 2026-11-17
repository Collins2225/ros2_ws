import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool


class HeaterServer(Node):

    def __init__(self):
        super().__init__('heater_server')
        self.srv = self.create_service(SetBool, 'heater_control', self.heater_callback)
        self.heater_on = False
        self.get_logger().info('Heater service server ready.')

    def heater_callback(self, request, response):
        # Only print/log when the state actually changes
        if request.data != self.heater_on:
            self.heater_on = request.data
            status = 'Heater:On' if self.heater_on else 'Heater:Off'
            print(status)
            self.get_logger().info(status)

        response.success = True
        response.message = 'Heater is now {}'.format(
            'On' if self.heater_on else 'Off'
        )
        return response


def main(args=None):
    rclpy.init(args=args)
    node = HeaterServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()