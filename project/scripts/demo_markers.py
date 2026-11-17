#!/usr/bin/env python3
"""Helper for the demo: draws the robot's trail, the goal circle and the algorithm name in RViz.
It only LISTENS to /odom and publishes pictures; it never drives the robot.
Run by demo.sh. The trail comes from the wheel odometry, so it can drift a little."""
import json
import math
import os
import sys

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray

GOAL = (float(os.environ.get('DEMO_GOAL_X', '3.0')), float(os.environ.get('DEMO_GOAL_Y', '0.0')))
GOAL_RADIUS = 0.15
SCENARIO_FILE = os.environ.get('DEMO_SCENARIO_FILE', '')


class DemoMarkers(Node):
    def __init__(self):
        super().__init__('demo_markers')
        self.label = os.environ.get('DEMO_LABEL', '')
        self.obstacles = []                      # the room's furniture and walls, drawn as grey shapes
        if SCENARIO_FILE and os.path.isfile(SCENARIO_FILE):
            try:
                with open(SCENARIO_FILE) as f:
                    self.obstacles = json.load(f)['obstacles']
            except (OSError, ValueError, KeyError) as err:
                self.get_logger().warning(f'could not read {SCENARIO_FILE}: {err}')
        top = max([o['y'] + (o['sy'] / 2 if o['shape'] == 'box' else o['r']) for o in self.obstacles] + [0.8])
        self.label_xy = (GOAL[0] / 2.0, top + 0.3)
        self.path = Path()
        self.path.header.frame_id = 'odom'
        self.last_xy = None
        self.path_pub = self.create_publisher(Path, '/demo/path', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/demo/markers', 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_timer(1.0, self.publish_markers)

    def on_odom(self, msg):
        p = msg.pose.pose.position
        if self.last_xy is not None and math.hypot(p.x - self.last_xy[0], p.y - self.last_xy[1]) < 0.02:
            return
        self.last_xy = (p.x, p.y)
        ps = PoseStamped()
        ps.header.frame_id = 'odom'
        ps.header.stamp = msg.header.stamp
        ps.pose = msg.pose.pose
        self.path.poses.append(ps)
        self.path.header.stamp = msg.header.stamp
        self.path_pub.publish(self.path)

    def marker(self, mid, kind, x, y, z, sx, sy, sz, rgba, text=''):
        m = Marker()
        m.header.frame_id = 'odom'          # stamp stays 0 = "use the newest transform"
        m.ns = 'demo'
        m.id = mid
        m.type = kind
        m.action = Marker.ADD
        m.pose.position.x, m.pose.position.y, m.pose.position.z = x, y, z
        m.pose.orientation.w = 1.0
        m.scale.x, m.scale.y, m.scale.z = sx, sy, sz
        m.color.r, m.color.g, m.color.b, m.color.a = rgba
        m.text = text
        return m

    def publish_markers(self):
        arr = MarkerArray()
        d = 2.0 * GOAL_RADIUS
        arr.markers.append(self.marker(0, Marker.CYLINDER, GOAL[0], GOAL[1], 0.01, d, d, 0.02, (0.1, 0.9, 0.2, 0.6)))
        arr.markers.append(self.marker(1, Marker.TEXT_VIEW_FACING, GOAL[0], GOAL[1], 0.25, 0.0, 0.0, 0.12,
                                       (1.0, 1.0, 1.0, 1.0), 'GOAL'))
        arr.markers.append(self.marker(2, Marker.TEXT_VIEW_FACING, self.label_xy[0], self.label_xy[1], 0.1,
                                       0.0, 0.0, 0.14, (1.0, 0.9, 0.2, 1.0), f'ALGORITHM: {self.label.upper()}'))
        for i, o in enumerate(self.obstacles):
            grey = (0.65, 0.65, 0.70, 0.55)
            if o['shape'] == 'box':
                m = self.marker(10 + i, Marker.CUBE, o['x'], o['y'], 0.15, o['sx'], o['sy'], 0.30, grey)
                m.pose.orientation.z = math.sin(o['yaw'] / 2.0)
                m.pose.orientation.w = math.cos(o['yaw'] / 2.0)
            else:
                m = self.marker(10 + i, Marker.CYLINDER, o['x'], o['y'], 0.15, 2.0 * o['r'], 2.0 * o['r'], 0.30, grey)
            arr.markers.append(m)
        self.marker_pub.publish(arr)


def main():
    rclpy.init(args=sys.argv)
    node = DemoMarkers()
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
