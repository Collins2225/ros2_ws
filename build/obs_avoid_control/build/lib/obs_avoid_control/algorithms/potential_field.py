"""Algorithm 5: artificial potential field (Khatib, 1986).

Idea: pretend the room is a hilly landscape.
  * the GOAL is a magnet that pulls the robot with a steady force,
  * every obstacle the sonar sees is a pushing hand: the closer it is, the harder it pushes.
The robot adds up all pulls and pushes into ONE arrow and simply drives the way the arrow
points. Nothing is remembered, nothing is planned: the arrow is recomputed every cycle.

Known weakness (this is a finding for the thesis, not a bug to hide): if the pushes and the
pull cancel out, the robot can get stuck or dither (a "local minimum").
"""
import math
import os

from obs_avoid_control.core.interface import Algorithm, Command


class Controller(Algorithm):
    name = 'potential_field'

    def __init__(self):
        self.cruise_speed = 0.20     # m/s, the console's speed limit
        self.turn_gain = 1.0         # rad/s of turning per radian of heading error
        self.max_turn = 0.6          # rad/s: gentle turning (the robot really turns ~0.7x what is asked)
        self.att_gain = 1.0          # strength of the goal's pull (constant, like a magnet)
        self.rep_gain = 0.02         # strength of the obstacles' push (eta in Khatib's formula)
        self.influence = 0.60        # m: obstacles farther than this do not push at all (rho_0)
        self.rep_cap = 3.0           # biggest push one single reading may give (keeps numbers sane)
        self.front_half = math.radians(20)
        self.slow_start = 0.60       # m: closer than this in front, the robot starts to slow down
        self.slow_end = 0.25         # m: at this distance (or closer) it creeps at min_speed
        self.min_speed = 0.04        # m/s: creeping speed
        self.debug = os.environ.get('ALGO_DEBUG') == '1'
        self.last_print = -10.0

    def reset(self):
        self.last_print = -10.0

    def step(self, p):
        # never drive blind: if nothing in front has been looked at recently, wait
        front = p.sector_min(-self.front_half, self.front_half)
        if math.isnan(front):
            return Command(0.0, 0.0)

        # --- the goal pulls: a fixed-length arrow towards the goal (robot frame: x ahead, y left)
        b = p.goal_bearing()
        fx = self.att_gain * math.cos(b)
        fy = self.att_gain * math.sin(b)

        # --- every sonar reading pushes: arrow pointing AWAY from the thing it saw
        #     push strength = eta * (1/r - 1/rho_0) / r^2   (Khatib's formula)
        for i, r in enumerate(p.ranges):
            if math.isnan(r) or r >= self.influence:
                continue
            r = max(r, 0.05)
            a = p.angle_min + i * p.angle_increment
            push = self.rep_gain * (1.0 / r - 1.0 / self.influence) / (r * r)
            push = min(push, self.rep_cap)
            fx -= push * math.cos(a)
            fy -= push * math.sin(a)

        # exact tie (a perfectly symmetric picture): break it to the LEFT, as the others do
        if abs(fy) < 1e-6:
            fy = 1e-6

        # --- drive the way the arrow points
        heading_error = math.atan2(fy, fx)           # positive = arrow points left
        angular = max(-self.max_turn, min(self.max_turn, self.turn_gain * heading_error))
        # speed: less when the arrow points sideways, less when something is close ahead,
        # and zero when the arrow points behind us (then the robot just turns on the spot)
        room = max(0.0, min(1.0, (front - self.slow_end) / (self.slow_start - self.slow_end)))
        speed_cap = self.min_speed + (self.cruise_speed - self.min_speed) * room
        linear = speed_cap * max(0.0, math.cos(heading_error))

        if self.debug and p.time - self.last_print >= 1.0:
            self.last_print = p.time
            print(f'[potential_field] t={p.time:.1f}s force=({fx:+.2f},{fy:+.2f}) '
                  f'heading_err={math.degrees(heading_error):+.0f}deg -> '
                  f'speed={linear:.2f} turn={angular:+.2f}', flush=True)

        return Command(linear, angular)
