"""Algorithm 3: Braitenberg-style avoidance.

Idea: no stopping, no decisions. Every obstacle the sonar sees "pushes" the robot
away from it. The closer the obstacle, the harder the push.

  obstacle on the LEFT   -> push the robot to turn RIGHT
  obstacle on the RIGHT  -> push the robot to turn LEFT
  obstacle straight ahead -> slow down (the pushes from both sides cancel)

The goal "pulls" the robot, but the pull gets weaker when an obstacle is close.
"""
import math
import os

from obs_avoid_control.core.interface import Algorithm, Command


class Controller(Algorithm):
    name = 'braitenberg'

    def __init__(self):
        self.cruise_speed = 0.20     # m/s, the console's speed limit
        self.min_speed = 0.04        # never fully stops (a Braitenberg vehicle keeps moving)
        self.far = 0.70              # m: obstacles farther than this are ignored
        self.near = 0.20             # m: obstacles this close push with full strength
        self.avoid_gain = 1.2        # rad/s of turning at full push (difference left vs right)
        self.goal_gain = 1.0         # rad/s of turning per radian of goal bearing
        self.debug = os.environ.get('ALGO_DEBUG') == '1'
        self.last_print = -10.0

    def reset(self):
        self.last_print = -10.0

    def push(self, r):
        """How strongly a reading r (metres) pushes: 0 = far away, 1 = very close."""
        if math.isnan(r):
            return 0.0
        return max(0.0, min(1.0, (self.far - r) / (self.far - self.near)))

    def step(self, p):
        left = 0.0     # strongest push from the left half of the picture
        right = 0.0    # strongest push from the right half
        ahead = 0.0    # strongest push from straight ahead (+-15 degrees)
        front_known = False

        for i, r in enumerate(p.ranges):
            a = p.angle_min + i * p.angle_increment
            if abs(a) <= math.radians(15) and not math.isnan(r):
                front_known = True
            strength = self.push(r) * math.cos(a)
            # Each "eye" listens mostly to its own side: the weight grows from 0 (dead
            # ahead) to 1 (25 degrees or more to the side). So an obstacle that sticks
            # out further on one side pushes that eye harder, and the robot turns away.
            side_weight = min(1.0, abs(a) / math.radians(25))
            if a > 0:
                left = max(left, strength * side_weight)
            elif a < 0:
                right = max(right, strength * side_weight)
            if abs(a) <= math.radians(15):
                ahead = max(ahead, strength)

        # never drive blind: if nothing in front has been looked at recently, wait
        if not front_known:
            return Command(0.0, 0.0)

        # left obstacle (positive angles) pushes the robot to turn right (negative)
        turn_away = self.avoid_gain * (right - left)
        # the goal pulls, but less so when something is close
        caring = 1.0 - max(left, right, ahead)
        turn_goal = caring * self.goal_gain * p.goal_bearing()

        speed = max(self.min_speed, self.cruise_speed * (1.0 - ahead))
        angular = turn_away + turn_goal

        if self.debug and p.time - self.last_print >= 1.0:
            self.last_print = p.time
            print(f'[braitenberg] t={p.time:.1f}s left={left:.2f} right={right:.2f} '
                  f'ahead={ahead:.2f} -> speed={speed:.2f} turn={angular:+.2f}', flush=True)

        return Command(speed, angular)
