import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64


class Probe(Node):
    def __init__(self):
        super().__init__('servo_probe')
        self.pub = self.create_publisher(Float64, '/servo_cmd', 10)
        self.create_subscription(JointState, '/joint_states', self.on_joints, 10)
        self.angle = None          # newest neck angle we heard
        self.samples = []          # list of (seconds since command, angle)
        self.t0 = None

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_joints(self, msg):
        if 'servo_joint' not in msg.name:
            return
        self.angle = msg.position[msg.name.index('servo_joint')]
        if self.t0 is not None:
            self.samples.append((self.now() - self.t0, self.angle))

    def run_step(self, target):
        start = self.angle
        self.samples = []
        self.t0 = self.now()
        self.pub.publish(Float64(data=target))
        while self.now() - self.t0 < 2.0:            # watch for 2 simulated seconds
            rclpy.spin_once(self, timeout_sec=0.05)
        self.t0 = None

        move = target - start
        t90 = next((t for t, a in self.samples if (a - start) / move >= 0.9), None)
        over = max((a - target) / move for _, a in self.samples)
        wrong = [t for t, a in self.samples if abs(a - target) > 0.02]
        settle = max(wrong) if wrong else 0.0
        self.get_logger().info(
            f'{start:+.2f} -> {target:+.2f} rad | 90% of the way after {t90:.2f} s | '
            f'overshoot {max(over, 0) * 100:.0f}% | settled (within 0.02 rad) after {settle:.2f} s | '
            f'final angle {self.samples[-1][1]:+.3f}')


def main():
    rclpy.init()
    node = Probe()
    while node.angle is None or node.pub.get_subscription_count() == 0:
        rclpy.spin_once(node, timeout_sec=0.1)       # wait until the neck's messages arrive
    for target in (0.6, -0.6, 0.0):
        node.run_step(target)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
