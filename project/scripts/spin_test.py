import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from tf2_msgs.msg import TFMessage

SPIN_SPEED = 0.8     # rad/s: the same turn speed the algorithm uses
SPIN_TIME = 3.0      # seconds of turning


class SpinTest(Node):
    def __init__(self):
        super().__init__('spin_test')
        self.pose = None
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(TFMessage, '/world/obstacle_course/dynamic_pose/info', self.on_pose, qos)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def on_pose(self, msg):
        t = msg.transforms[0].transform          # entry 0 = the robot's TRUE pose
        q = t.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.pose = (t.translation.x, t.translation.y, yaw)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def spin_and_measure(self, angular, seconds):
        start = self.pose
        last_yaw, turned, max_drift, last_send = start[2], 0.0, 0.0, -1.0
        end = self.now() + seconds
        while self.now() < end and rclpy.ok():
            if self.now() - last_send >= 0.05:            # send the command 20 times a second
                cmd = Twist()
                cmd.angular.z = angular
                self.pub.publish(cmd)
                last_send = self.now()
            rclpy.spin_once(self, timeout_sec=0.01)
            x, y, yaw = self.pose
            turned += math.atan2(math.sin(yaw - last_yaw), math.cos(yaw - last_yaw))   # add up the turning
            last_yaw = yaw
            max_drift = max(max_drift, math.hypot(x - start[0], y - start[1]))
        self.pub.publish(Twist())                          # stop
        settle = self.now() + 1.5
        while self.now() < settle and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
        x, y, _ = self.pose
        final_drift = math.hypot(x - start[0], y - start[1])
        print(f'commanded turn rate : {angular:+.2f} rad/s for {seconds:.1f} s')
        print(f'achieved turn rate  : {turned / seconds:+.2f} rad/s   ({math.degrees(turned):+.0f} degrees in total)')
        print(f"robot's middle moved: up to {max_drift * 100:.1f} cm during the spin, {final_drift * 100:.1f} cm at the end")
        print()


def main():
    rclpy.init()
    node = SpinTest()
    while (node.pose is None or node.now() == 0.0) and rclpy.ok():   # wait for the pose AND the clock
        rclpy.spin_once(node, timeout_sec=0.1)
    for w in (SPIN_SPEED, -SPIN_SPEED):                    # once to the left, once to the right
        node.spin_and_measure(w, SPIN_TIME)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
