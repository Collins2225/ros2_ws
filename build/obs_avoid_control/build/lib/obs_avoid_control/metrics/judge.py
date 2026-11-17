import csv
import math
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from tf2_msgs.msg import TFMessage

# The arena. These numbers MUST match worlds/obstacle_course.sdf.
# (Later we will make ONE file that builds the world AND feeds the referee.)
OBSTACLES = [
    {'name': 'box_1',    'shape': 'box',    'x': 1.0, 'y': 0.0,  'sx': 0.30, 'sy': 0.30, 'yaw': 0.0},
    {'name': 'box_2',    'shape': 'box',    'x': 1.8, 'y': 0.6,  'sx': 0.40, 'sy': 0.20, 'yaw': 0.4},
    {'name': 'pillar_1', 'shape': 'circle', 'x': 2.2, 'y': -0.5, 'r': 0.15},
    {'name': 'wall_1',   'shape': 'box',    'x': 3.5, 'y': 0.0,  'sx': 0.10, 'sy': 3.00, 'yaw': 0.0},
]

# The robot's body seen from above, in metres, measured from its middle:
ROBOT_X0, ROBOT_X1 = -0.10, 0.11      # back edge, front edge
ROBOT_Y0, ROBOT_Y1 = -0.082, 0.082    # right edge, left edge


def rotate(px, py, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return c * px - s * py, s * px + c * py


def box_corners(cx, cy, x0, x1, y0, y1, yaw):
    """The 4 corners of a rectangle, placed at (cx, cy) and turned by yaw."""
    corners = []
    for px, py in ((x0, y0), (x1, y0), (x1, y1), (x0, y1)):
        rx, ry = rotate(px, py, yaw)
        corners.append((cx + rx, cy + ry))
    return corners


def point_segment_distance(p, a, b):
    """How far is point p from the line segment a-b?"""
    dx, dy = b[0] - a[0], b[1] - a[1]
    length2 = dx * dx + dy * dy
    t = 0.0 if length2 == 0 else max(0.0, min(1.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / length2))
    return math.hypot(p[0] - (a[0] + t * dx), p[1] - (a[1] + t * dy))


def separated(P, Q):
    """True if we can slide a straight line between shapes P and Q (they do not touch)."""
    for poly in (P, Q):
        for i in range(len(poly)):
            x1, y1 = poly[i]
            x2, y2 = poly[(i + 1) % len(poly)]
            nx, ny = y1 - y2, x2 - x1                    # a direction pointing away from this edge
            a = [nx * x + ny * y for x, y in P]
            b = [nx * x + ny * y for x, y in Q]
            if max(a) < min(b) or max(b) < min(a):
                return True
    return False


def polygon_gap(P, Q):
    """Free space between two rectangles. 0 means touching or overlapping."""
    if not separated(P, Q):
        return 0.0
    best = math.inf
    for A, B in ((P, Q), (Q, P)):
        for corner in A:
            for i in range(len(B)):
                best = min(best, point_segment_distance(corner, B[i], B[(i + 1) % len(B)]))
    return best


def polygon_circle_gap(P, centre, radius):
    """Free space between a rectangle and a round pillar."""
    signs = []
    for i in range(len(P)):
        x1, y1 = P[i]
        x2, y2 = P[(i + 1) % len(P)]
        signs.append((x2 - x1) * (centre[1] - y1) - (y2 - y1) * (centre[0] - x1))
    if all(s >= 0 for s in signs) or all(s <= 0 for s in signs):
        return 0.0                                       # the pillar centre is inside the robot
    d = min(point_segment_distance(centre, P[i], P[(i + 1) % len(P)]) for i in range(len(P)))
    return max(0.0, d - radius)


class Judge(Node):
    def __init__(self):
        super().__init__('judge')
        self.declare_parameter('label', 'unnamed')
        self.declare_parameter('goal_x', 3.0)
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('goal_tolerance', 0.15)
        self.declare_parameter('timeout', 120.0)
        self.declare_parameter('stop_on_collision', True)
        self.declare_parameter('csv_path', '~/ros2_ws/project/results/raw/runs.csv')
        self.declare_parameter('scenario_file', '')    # a .json with the obstacles; empty = the built-in arena
        g = self.get_parameter
        self.label = g('label').value
        self.goal = (g('goal_x').value, g('goal_y').value)
        self.tolerance = g('goal_tolerance').value
        self.timeout = g('timeout').value
        self.stop_on_collision = g('stop_on_collision').value
        self.csv_path = os.path.expanduser(g('csv_path').value)
        self.obstacles = OBSTACLES
        scenario_file = os.path.expanduser(g('scenario_file').value)
        if scenario_file:
            import json
            with open(scenario_file) as f:
                self.obstacles = json.load(f)['obstacles']
            self.get_logger().info(f'obstacles from {scenario_file}: {len(self.obstacles)} objects')

        # Work out each obstacle's outline once, at the start.
        self.shapes = []
        for o in self.obstacles:
            if o['shape'] == 'box':
                poly = box_corners(o['x'], o['y'], -o['sx'] / 2, o['sx'] / 2, -o['sy'] / 2, o['sy'] / 2, o['yaw'])
                self.shapes.append(('box', poly, o['name']))
            else:
                self.shapes.append(('circle', (o['x'], o['y']), o['r'], o['name']))

        self.latest = None          # newest TRUE pose: (x, y, yaw)
        self.start_pose = None
        self.t0 = None              # clock starts when the robot first moves
        self.last = None
        self.path = 0.0
        self.collisions = 0
        self.first_collision_t = None
        self.in_contact = False
        self.min_gap = math.inf
        self.min_gap_info = ''
        self.done = False
        self.warned = False
        self.trace = []             # the robot's path: (time, x, y, yaw, gap)

        # The bridge sends about 1000 messages a second. We only need the newest one.
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(TFMessage, '/world/obstacle_course/dynamic_pose/info', self.on_pose, qos)
        self.create_timer(0.02, self.tick)        # the referee looks 50 times a second
        self.get_logger().info(f"referee started for '{self.label}'")

    def on_pose(self, msg):
        tfs = msg.transforms
        if len(tfs) < 3:
            return
        b = tfs[1].transform.translation      # entry 2 (base_footprint) should sit at zero
        if not self.warned and abs(b.x) + abs(b.y) + abs(b.z) > 1e-3:
            self.warned = True
            self.get_logger().warning('the pose list looks different than expected - is entry 0 still the robot?')
        p, q = tfs[0].transform.translation, tfs[0].transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.latest = (p.x, p.y, yaw)

    def closest_gap(self, x, y, yaw):
        robot = box_corners(x, y, ROBOT_X0, ROBOT_X1, ROBOT_Y0, ROBOT_Y1, yaw)
        best, who = math.inf, ''
        for s in self.shapes:
            gap = polygon_gap(robot, s[1]) if s[0] == 'box' else polygon_circle_gap(robot, s[1], s[2])
            if gap < best:
                best, who = gap, s[-1]
        return best, who

    def tick(self):
        if self.done or self.latest is None:
            return
        x, y, yaw = self.latest
        now = self.get_clock().now().nanoseconds * 1e-9

        if self.start_pose is None:
            self.start_pose = (x, y)
            self.get_logger().info(f'robot is at ({x:.2f}, {y:.2f}). Waiting for it to move...')
        if self.t0 is None:
            if math.hypot(x - self.start_pose[0], y - self.start_pose[1]) < 0.02:
                return
            self.t0, self.last = now, self.start_pose
            self.get_logger().info('robot started moving - the clock is running')

        elapsed = now - self.t0
        self.path += math.hypot(x - self.last[0], y - self.last[1])
        self.last = (x, y)

        gap, who = self.closest_gap(x, y, yaw)
        if gap < self.min_gap:
            self.min_gap = gap
            self.min_gap_info = f'{who} at {elapsed:.1f} s, robot at ({x:.2f}, {y:.2f})'
        if not self.trace or elapsed - self.trace[-1][0] >= 0.1:
            self.trace.append((elapsed, x, y, yaw, gap))      # one point every 0.1 s
        touching = gap <= 0.0
        if touching and not self.in_contact:
            self.collisions += 1
            if self.first_collision_t is None:
                self.first_collision_t = elapsed
            self.get_logger().warning(f'TOUCH! at {elapsed:.1f} s, position ({x:.2f}, {y:.2f})')
        self.in_contact = touching

        goal_dist = math.hypot(self.goal[0] - x, self.goal[1] - y)
        if touching and self.stop_on_collision:
            self.finish('COLLISION', elapsed, goal_dist)
        elif goal_dist < self.tolerance:
            self.finish('SUCCESS' if self.collisions == 0 else 'REACHED_WITH_COLLISION', elapsed, goal_dist)
        elif elapsed > self.timeout:
            self.finish('TIMEOUT', elapsed, goal_dist)
        else:
            self.get_logger().info(f'{elapsed:5.1f} s  true position ({x:.2f}, {y:.2f})  closest gap {gap:.2f} m',
                                   throttle_duration_sec=5.0)

    def finish(self, outcome, elapsed, goal_dist):
        self.done = True
        straight = math.hypot(self.goal[0] - self.start_pose[0], self.goal[1] - self.start_pose[1])
        efficiency = min(1.0, max(straight - self.tolerance, 0.0) / self.path) if self.path > 0 else 0.0
        success = outcome == 'SUCCESS'
        report = [
            '============== REFEREE REPORT ==============',
            f' label            : {self.label}',
            f' outcome          : {outcome}',
            f' time             : {elapsed:.1f} s',
            f' distance driven  : {self.path:.2f} m   (straight line would be {straight:.2f} m)',
            f' path efficiency  : {efficiency:.2f}',
            f' collisions       : {self.collisions}',
            f' closest approach : {self.min_gap:.3f} m   ({self.min_gap_info})',
            f' final goal dist  : {goal_dist:.2f} m',
            '============================================',
        ]
        for line in report:
            self.get_logger().info(line)

        os.makedirs(os.path.dirname(self.csv_path), exist_ok=True)
        new_file = not os.path.exists(self.csv_path)
        with open(self.csv_path, 'a', newline='') as f:
            w = csv.writer(f)
            if new_file:
                w.writerow(['label', 'outcome', 'success', 'time_s', 'path_m', 'straight_m', 'efficiency',
                            'collisions', 'first_collision_s', 'min_gap_m', 'final_goal_dist_m', 'closest_obstacle'])
            w.writerow([self.label, outcome, int(success), f'{elapsed:.2f}', f'{self.path:.3f}', f'{straight:.3f}',
                        f'{efficiency:.3f}', self.collisions,
                        '' if self.first_collision_t is None else f'{self.first_collision_t:.2f}',
                        f'{self.min_gap:.3f}', f'{goal_dist:.3f}', self.min_gap_info.split(' at ')[0]])
        traj_dir = os.path.join(os.path.dirname(self.csv_path), 'trajectories')
        os.makedirs(traj_dir, exist_ok=True)
        traj_file = os.path.join(traj_dir, f"{self.label}_{time.strftime('%Y%m%d_%H%M%S')}.csv")
        with open(traj_file, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['t_s', 'x_m', 'y_m', 'yaw_rad', 'gap_m'])
            for r in self.trace:
                w.writerow([f'{r[0]:.2f}', f'{r[1]:.4f}', f'{r[2]:.4f}', f'{r[3]:.4f}', f'{r[4]:.4f}'])
        self.get_logger().info(f'saved the path to {traj_file}')
        self.get_logger().info(f'saved one row to {self.csv_path}. You can press Ctrl+C now.')


def main():
    rclpy.init()
    node = Judge()
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
