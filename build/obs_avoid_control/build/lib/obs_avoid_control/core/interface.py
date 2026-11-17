import importlib
import math
from dataclasses import dataclass


def wrap_angle(a):
    """Turn any angle into the range -pi .. +pi."""
    return math.atan2(math.sin(a), math.cos(a))


@dataclass
class Command:
    """What an algorithm answers: two numbers."""
    linear: float = 0.0     # forward speed, metres per second
    angular: float = 0.0    # turning speed, radians per second (positive = turn left)


@dataclass
class Perception:
    """Everything the robot knows at this moment."""
    ranges: list            # 31 distances (metres). math.nan means "never looked there"
    angle_min: float        # angle of the first slot (radians, negative = right)
    angle_increment: float  # angle between neighbouring slots
    x: float                # robot position (metres)
    y: float
    yaw: float              # where the nose points (radians, 0 = along +x, positive = left)
    goal_x: float
    goal_y: float
    time: float             # seconds since the run started

    def goal_distance(self):
        return math.hypot(self.goal_x - self.x, self.goal_y - self.y)

    def goal_bearing(self):
        """Angle to the goal, measured from the robot's nose. Positive = goal is on the left."""
        return wrap_angle(math.atan2(self.goal_y - self.y, self.goal_x - self.x) - self.yaw)

    def _known(self, lo, hi):
        out = []
        for i, r in enumerate(self.ranges):
            a = self.angle_min + i * self.angle_increment
            if lo - 1e-6 <= a <= hi + 1e-6 and not math.isnan(r):
                out.append(r)
        return out

    def sector_min(self, lo, hi):
        """Closest known distance between two angles (radians). nan if nothing is known there."""
        v = self._known(lo, hi)
        return min(v) if v else math.nan

    def sector_mean(self, lo, hi):
        """Average known distance between two angles. Big number = lots of room."""
        v = self._known(lo, hi)
        return sum(v) / len(v) if v else math.nan


class Algorithm:
    """Every avoidance method is a class called Controller that follows this shape."""
    name = 'base'

    def reset(self):
        """Called once before a run starts. Forget everything remembered."""

    def step(self, p):
        """Look at Perception p and return a Command."""
        raise NotImplementedError


def load_algorithm(name):
    """The FILE NAME is the algorithm's name: 'stop_and_turn' loads algorithms/stop_and_turn.py"""
    module = importlib.import_module(f'obs_avoid_control.algorithms.{name}')
    return module.Controller()
