import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class NeckProbe(Node):
    def __init__(self):
        super().__init__('neck_probe')
        self.data = []          # (time, neck angle) pairs
        self.idx = None
        self.create_subscription(JointState, '/joint_states', self.on_joints, 100)

    def on_joints(self, msg):
        if self.idx is None:
            if 'servo_joint' not in msg.name:
                return
            self.idx = msg.name.index('servo_joint')
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.data.append((t, msg.position[self.idx]))


def main():
    rclpy.init()
    node = NeckProbe()
    start = None
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.data:
            if start is None:
                start = node.data[0][0]
            if node.data[-1][0] - start > 9.0:      # listen for 9 simulated seconds
                break
    d = [x for x in node.data if x[0] - start > 1.0]   # skip the first second
    angles = [a for _, a in d]
    hi, lo = max(angles), min(angles)
    ups = [d[i][0] for i in range(1, len(d)) if d[i - 1][1] < 0 <= d[i][1]]
    print(f'highest angle: {hi:+.3f} rad ({hi * 57.3:+.0f} deg)')
    print(f'lowest angle:  {lo:+.3f} rad ({lo * 57.3:+.0f} deg)')
    if len(ups) >= 2:
        period = (ups[-1] - ups[0]) / (len(ups) - 1)
        print(f'one full there-and-back swing takes {period:.2f} s')
        print(f'average neck speed: {2 * (hi - lo) / period:.2f} rad/s (we asked for 3.00)')
    else:
        print('not enough swings seen - is sweep_scanner running?')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
