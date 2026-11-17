"""Where am I? Wheels count the DISTANCE, a gyroscope tells the HEADING.

Wheel odometry is good at "how far did I roll" but bad at "how much did I turn" on a
skid-steer robot, because the wheels slip sideways while turning. A gyroscope measures
the turning directly, so slip does not fool it.

So we mix them (dead reckoning):
    heading  = start heading + sum of (gyro turn rate x time)
    position = sum of (distance the wheels rolled) in the direction of that heading
"""
import math


def wrap_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


class GyroOdometryFusion:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.yaw = None            # set from the very first wheel-odometry message
        self.last_odom_xy = None
        self.yaw_at_last_odom = None
        self.last_imu_t = None
        self.imu_count = 0
        self.max_gap = 0.5         # s: a longer silence between gyro samples is not integrated

    def update_gyro(self, t, wz):
        """t: time of the sample (s); wz: turn rate around the vertical axis (rad/s, + = left)."""
        if self.last_imu_t is not None and self.yaw is not None:
            dt = t - self.last_imu_t
            if 0.0 < dt < self.max_gap:
                self.yaw = wrap_angle(self.yaw + wz * dt)
        self.last_imu_t = t
        self.imu_count += 1

    def update_odom(self, ox, oy, oyaw):
        """Wheel odometry: only the DISTANCE rolled since the last message is used."""
        if self.yaw is None:
            self.yaw = oyaw        # both start with the same heading and position
            self.x, self.y = ox, oy
        elif self.last_odom_xy is not None:
            ds = math.hypot(ox - self.last_odom_xy[0], oy - self.last_odom_xy[1])
            # use the middle of the heading change during this step
            mid = self.yaw_at_last_odom + 0.5 * wrap_angle(self.yaw - self.yaw_at_last_odom)
            self.x += ds * math.cos(mid)
            self.y += ds * math.sin(mid)
        self.last_odom_xy = (ox, oy)
        self.yaw_at_last_odom = self.yaw

    @property
    def ready(self):
        # need wheel odometry AND a few gyro samples, otherwise the heading would silently stay frozen
        return self.yaw is not None and self.imu_count >= 5

    def pose(self):
        return (self.x, self.y, self.yaw) if self.ready else None
