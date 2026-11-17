#!/usr/bin/env python3
"""record_odom.py <file.csv>  - writes time, x, y, yaw of the robot (from /odom) five times a second.
Run with:  /usr/bin/python3 record_odom.py out.csv --ros-args -p use_sim_time:=true"""
import math, sys
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class Rec(Node):
    def __init__(self, path):
        super().__init__('odom_recorder')
        self.f = open(path, 'w')
        self.f.write('t,x,y,yaw\n')
        self.last = -1.0
        self.create_subscription(Odometry, '/odom', self.cb, 10)
    def cb(self, m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if t - self.last < 0.2:
            return
        self.last = t
        q = m.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.f.write(f'{t:.3f},{m.pose.pose.position.x:.4f},{m.pose.pose.position.y:.4f},{yaw:.4f}\n')
        self.f.flush()

def main():
    rclpy.init(args=sys.argv)
    node = Rec(sys.argv[1])
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

main()
