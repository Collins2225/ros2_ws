import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu, LaserScan

from obs_avoid_control.core.interface import Perception, load_algorithm
from obs_avoid_control.core.pose_fusion import GyroOdometryFusion


class AvoidanceNode(Node):
    def __init__(self):
        super().__init__('avoidance_node')
        self.declare_parameter('algorithm', 'stop_and_turn')
        self.declare_parameter('goal_x', 3.0)
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('goal_tolerance', 0.10)   # "arrived" when this close (m)
        self.declare_parameter('max_linear', 0.2)        # same speed limit for every algorithm
        self.declare_parameter('max_angular', 1.0)
        self.declare_parameter('timeout', 120.0)         # give up after this many seconds
        self.declare_parameter('rate_hz', 20.0)
        # 'gyro' = wheels measure distance, gyroscope measures heading (default, used for all algorithms)
        # 'odom' = wheels only (the old behaviour; it drifts when the robot turns)
        self.declare_parameter('pose_source', 'gyro')

        p = self.get_parameter
        self.goal = (p('goal_x').value, p('goal_y').value)
        self.tolerance = p('goal_tolerance').value
        self.max_linear = p('max_linear').value
        self.max_angular = p('max_angular').value
        self.timeout = p('timeout').value
        self.pose_source = p('pose_source').value
        if self.pose_source not in ('gyro', 'odom'):
            raise ValueError("pose_source must be 'gyro' or 'odom'")

        self.algo = load_algorithm(p('algorithm').value)
        self.algo.reset()
        self.scan = None          # newest picture from /scan_wide
        self.pose = None          # newest (x, y, yaw) the brain believes in
        self.wheel_yaw = None     # heading according to the wheels alone (only for the log)
        self.fusion = GyroOdometryFusion()
        self.t_start = None
        self.finished = False

        self.create_subscription(LaserScan, '/scan_wide', self.on_scan, 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        if self.pose_source == 'gyro':
            self.create_subscription(Imu, '/imu', self.on_imu, 50)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_timer(1.0 / p('rate_hz').value, self.tick)
        self.get_logger().info(
            f"running algorithm '{p('algorithm').value}', goal {self.goal}, pose source '{self.pose_source}'")

    def on_scan(self, msg):
        self.scan = msg

    def on_imu(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if t == 0.0:                                   # no timestamp: use our own clock
            t = self.get_clock().now().nanoseconds * 1e-9
        self.fusion.update_gyro(t, msg.angular_velocity.z)
        self.pose = self.fusion.pose()

    def on_odom(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        x, y = msg.pose.pose.position.x, msg.pose.pose.position.y
        self.wheel_yaw = yaw
        if self.pose_source == 'odom':
            self.pose = (x, y, yaw)
        else:
            self.fusion.update_odom(x, y, yaw)
            self.pose = self.fusion.pose()

    def tick(self):
        if self.finished or self.scan is None:
            return
        if self.pose is None:
            self.get_logger().warn('waiting for /odom' + (' and /imu' if self.pose_source == 'gyro' else '') + ' ...',
                                   throttle_duration_sec=3.0)
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.t_start is None:
            self.t_start = now
        elapsed = now - self.t_start

        s = self.scan
        # inf means "looked, nothing there": treat as the maximum range. nan stays nan.
        ranges = [s.range_max if math.isinf(r) else r for r in s.ranges]
        x, y, yaw = self.pose
        percept = Perception(ranges, s.angle_min, s.angle_increment, x, y, yaw,
                             self.goal[0], self.goal[1], elapsed)

        distance = percept.goal_distance()
        if distance < self.tolerance:
            self.finish(f'GOAL REACHED in {elapsed:.1f} s (the brain believes: x={x:.2f} y={y:.2f} '
                        f'heading={math.degrees(yaw):.0f} deg)')
            return
        if elapsed > self.timeout:
            self.finish(f'TIMEOUT after {elapsed:.1f} s, still {distance:.2f} m from the goal')
            return

        cmd = self.algo.step(percept)
        self.send(cmd.linear, cmd.angular)
        wheels = '' if self.wheel_yaw is None else f' (wheels alone say {math.degrees(self.wheel_yaw):.0f})'
        self.get_logger().info(
            f'{elapsed:5.1f} s   position ({x:.2f}, {y:.2f})   heading {math.degrees(yaw):.0f} deg{wheels}'
            f'   goal {distance:.2f} m away', throttle_duration_sec=2.0)

    def send(self, linear, angular):
        # The console enforces the rules, so no algorithm can break them.
        if not math.isfinite(linear):
            linear = 0.0
        if not math.isfinite(angular):
            angular = 0.0
        msg = Twist()
        msg.linear.x = max(0.0, min(linear, self.max_linear))
        msg.angular.z = max(-self.max_angular, min(angular, self.max_angular))
        self.cmd_pub.publish(msg)

    def finish(self, text):
        self.finished = True
        self.send(0.0, 0.0)
        self.get_logger().info(text)


def main():
    rclpy.init()
    node = AvoidanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.send(0.0, 0.0)
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
