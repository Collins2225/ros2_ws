import math
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import JointState, LaserScan
from std_msgs.msg import Float64


def to_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class SweepScanner(Node):
    def __init__(self):
        super().__init__('sweep_scanner')

        # Settings you can change from the command line without touching the code.
        self.declare_parameter('sweep_rate', 3.0)    # how fast the neck swings (rad/s)
        self.declare_parameter('sweep_limit', 1.4)   # how far it swings each way (rad)
        self.declare_parameter('max_age', 2.5)       # forget readings older than this (s)
        self.sweep_rate = self.get_parameter('sweep_rate').value
        self.sweep_limit = self.get_parameter('sweep_limit').value
        self.max_age = self.get_parameter('max_age').value

        # THE PICTURE: 31 slots of 5 degrees, from -75 to +75 degrees.
        self.bin_width = math.radians(5.0)
        self.n_bins = 31
        self.first_angle = -math.radians(75.0)
        self.ranges = [math.nan] * self.n_bins       # nan = "never seen"
        self.seen_at = [-1e9] * self.n_bins          # when each slot was last updated
        self.last_stamp = 0.0                        # time of the newest sonar ping

        # THE DIARY: (time, neck angle) pairs, newest last.
        self.neck_diary = deque(maxlen=300)
        self.servo_index = None

        # The neck target moves steadily back and forth.
        self.setpoint = 0.0
        self.direction = 1.0
        self.dt = 0.02

        self.create_subscription(JointState, '/joint_states', self.on_joints,
                                 QoSProfile(depth=100))
        self.create_subscription(LaserScan, '/sonar', self.on_sonar, 10)
        self.cmd_pub = self.create_publisher(Float64, '/servo_cmd', 10)
        self.scan_pub = self.create_publisher(LaserScan, '/scan_wide', 10)
        self.create_timer(self.dt, self.move_neck)           # 50 times a second
        self.create_timer(0.1, self.publish_picture)         # 10 times a second
        self.get_logger().info('sweep_scanner started')

    def on_joints(self, msg):
        # Write the neck angle into the diary.
        if self.servo_index is None:
            if 'servo_joint' not in msg.name:
                return
            self.servo_index = msg.name.index('servo_joint')
        self.neck_diary.append((to_seconds(msg.header.stamp),
                                msg.position[self.servo_index]))

    def on_sonar(self, msg):
        if len(self.neck_diary) < 2:
            return
        t = to_seconds(msg.header.stamp)
        self.last_stamp = t

        # Look up where the neck pointed at the moment of this ping.
        times = [d[0] for d in self.neck_diary]
        angles = [d[1] for d in self.neck_diary]
        neck = float(np.interp(t, times, angles))

        # Each of the 5 rays points at (neck angle + its own small angle).
        best = {}
        for i, r in enumerate(msg.ranges):
            beam = neck + msg.angle_min + i * msg.angle_increment
            k = round((beam - self.first_angle) / self.bin_width)
            if 0 <= k < self.n_bins:
                best[k] = min(best.get(k, math.inf), r)   # two rays in one slot: keep the closer

        for k, r in best.items():
            self.ranges[k] = r
            self.seen_at[k] = t

    def move_neck(self):
        # Slide the target steadily, and turn around at the ends.
        self.setpoint += self.direction * self.sweep_rate * self.dt
        if abs(self.setpoint) >= self.sweep_limit:
            self.setpoint = math.copysign(self.sweep_limit, self.setpoint)
            self.direction = -self.direction
        self.cmd_pub.publish(Float64(data=self.setpoint))

    def publish_picture(self):
        out = LaserScan()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'base_link'
        out.angle_min = self.first_angle
        out.angle_increment = self.bin_width
        out.angle_max = self.first_angle + (self.n_bins - 1) * self.bin_width
        out.range_min = 0.02
        out.range_max = 4.0
        out.scan_time = 4.0 * self.sweep_limit / self.sweep_rate   # one full there-and-back
        # Slots not refreshed for a while become "unknown" (nan), never "safe".
        out.ranges = [math.nan if (self.last_stamp - seen) > self.max_age else r
                      for r, seen in zip(self.ranges, self.seen_at)]
        self.scan_pub.publish(out)


def main():
    rclpy.init()
    node = SweepScanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
