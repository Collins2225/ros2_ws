import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class HelloRobot(Node):
    def __init__(self):
        super().__init__('hello_robot')

        self.stop_distance = 0.40       # stop when something is closer than this (metres)
        self.latest_distance = None     # we have not heard the sonar yet

        # EARS: listen to /sonar. Call self.on_sonar every time a message arrives.
        self.create_subscription(LaserScan, '/sonar', self.on_sonar, 10)

        # MOUTH: talk to /cmd_vel.
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # ALARM CLOCK: call self.decide 20 times every second.
        self.create_timer(0.05, self.decide)

        self.get_logger().info('hello_robot started')

    def on_sonar(self, msg):
        # msg.ranges is a list of 5 distances. Infinity means "saw nothing".
        seen = [r for r in msg.ranges if math.isfinite(r)]
        self.latest_distance = min(seen) if seen else msg.range_max

    def decide(self):
        cmd = Twist()                         # a blank note: all zeros = stand still
        if self.latest_distance is None:
            pass                              # nothing heard yet, so stay still
        elif self.latest_distance > self.stop_distance:
            cmd.linear.x = 0.2                # path is clear: drive forward
        # otherwise cmd stays zero: STOP

        self.cmd_pub.publish(cmd)
        self.get_logger().info(f'closest thing: {self.latest_distance} m',
                               throttle_duration_sec=1.0)


def main():
    rclpy.init()
    node = HelloRobot()
    try:
        rclpy.spin(node)                      # keep the program alive, listening
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.cmd_pub.publish(Twist())     # on Ctrl+C, tell the robot to stop
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
