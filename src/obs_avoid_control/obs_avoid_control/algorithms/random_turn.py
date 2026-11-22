import math
import os
import random

from obs_avoid_control.core.interface import Algorithm, Command


class Controller(Algorithm):
    """Algorithm 2: when blocked, turn a RANDOM amount in a RANDOM direction."""
    name = 'random_turn'

    def __init__(self):
        self.stop_distance = 0.40
        self.cruise_speed = 0.20
        self.turn_speed = 0.8
        self.steer_gain = 1.0
        self.seed = int(os.environ.get('ALGO_SEED', '0'))   # same seed = same dice rolls
        self.rng = random.Random(self.seed)
        self.turn_until = None
        self.turn_dir = 0

    def reset(self):
        self.rng = random.Random(self.seed)
        print(f'[random_turn] seed = {self.seed}', flush=True)
        self.turn_until = None
        self.turn_dir = 0

    def step(self, p):
        front = p.sector_min(math.radians(-15), math.radians(15))
        if math.isnan(front):
            return Command(0.0, 0.0)
        if self.turn_until is not None:
            if p.time < self.turn_until:
                return Command(0.0, self.turn_dir * self.turn_speed)
            self.turn_until = None
        if front < self.stop_distance:
            self.turn_dir = self.rng.choice((-1, 1))
            angle = self.rng.uniform(math.radians(30), math.radians(150))
            self.turn_until = p.time + angle / self.turn_speed
            print(f"[random_turn] t={p.time:.1f}s blocked at {front:.2f} m -> dice: turn {'LEFT' if self.turn_dir == 1 else 'RIGHT'} "
                  f"for {math.degrees(angle):.0f} deg (about {angle / self.turn_speed:.1f} s)", flush=True)
            return Command(0.0, self.turn_dir * self.turn_speed)
        turn = max(-self.turn_speed, min(self.turn_speed, self.steer_gain * p.goal_bearing()))
        return Command(self.cruise_speed, turn)
"""
bash ~/ros2_ws/project/scripts/house_diagrams.sh vfh
bash ~/ros2_ws/project/scripts/house_diagrams.sh vfh_plus
bash ~/ros2_ws/project/scripts/house_diagrams.sh nearness_diagram
"""