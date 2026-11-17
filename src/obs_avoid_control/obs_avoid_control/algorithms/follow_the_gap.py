"""Algorithm 6: Follow the Gap (goal-seeking version).

Idea: look at the sonar picture and find the GAPS, the stretches of directions where there
is free room. Then drive towards the gap that lets the robot head closest to the goal.

Steps (every 0.05 s):
  1. Read the sonar picture: one distance for every 5 degrees, from -75 to +75 degrees.
  2. Mark a direction as BLOCKED if something is closer than `free_dist` there.
  3. Make every obstacle "fatter" by the robot's size: a blocked direction also blocks its
     neighbours (an obstacle that is close covers a wider angle than a far one).
  4. A GAP is a run of neighbouring directions that are all free. Ignore gaps that are too narrow.
  5. For every gap, find the direction inside it that is closest to the goal direction
     (kept `edge_margin` away from the gap's edges). Pick the gap with the best score: the direction
     nearest the goal, with a bonus for a gap you can see far down (a deep gap beats a dead-end pocket).
     A small bonus for the side chosen last time stops the robot from dithering left and right.
  6. Turn towards that direction, and drive slower when the way is sharp or the front is close.
  7. If there is no gap at all, turn on the spot towards the side with more room.
"""
import math
import os

from obs_avoid_control.core.interface import Algorithm, Command


class Controller(Algorithm):
    name = 'follow_the_gap'

    def __init__(self):
        self.cruise_speed = 0.20       # m/s, the console's speed limit
        self.min_speed = 0.05          # m/s, while a gap is open the robot keeps creeping forward
        self.free_dist = 0.60          # m: closer than this = blocked direction
        self.look_max = 1.50           # m: readings are cut off here (a far wall and a farther wall look the same)
        self.bubble = 0.20             # m: robot radius plus a safety margin; obstacles grow by this much
        self.min_gap_slots = 2         # a gap must be at least this many neighbouring directions wide
        self.edge_margin = 1           # directions kept between the chosen heading and a gap edge
        self.turn_gain = 1.6           # rad/s of turning per radian of heading error
        self.spin_speed = 0.7          # rad/s when turning on the spot
        self.sharp = math.radians(50)  # heading errors above this: turn on the spot, do not drive
        self.side_bonus = 0.25         # rad: penalty for switching to the other side
        self.depth_weight = float(os.environ.get('FTG_DEPTH', '0.6'))   # rad: how much a deep (far-reaching) gap is preferred
        self.debug = os.environ.get('ALGO_DEBUG') == '1'
        self.last_print = -10.0
        self.reset()

    def reset(self):
        self.last_print = -10.0
        self.last_side = 0             # +1 = last chosen heading was left, -1 = right, 0 = none yet
        self.spin_dir = 0              # direction of the current spin-on-the-spot (0 = not spinning)

    # ------------------------------------------------------------------ helpers
    def blocked_map(self, p):
        """True for every direction that is blocked (after making obstacles fatter)."""
        n = len(p.ranges)
        rs = []
        for r in p.ranges:
            rs.append(self.look_max if math.isnan(r) else min(r, self.look_max))
        blocked = [r < self.free_dist for r in rs]
        grown = list(blocked)
        for i, r in enumerate(rs):
            if not blocked[i]:
                continue
            half = math.asin(min(1.0, self.bubble / max(r, 0.05)))      # how wide the obstacle looks, in radians
            k = int(math.ceil(half / p.angle_increment))
            for j in range(max(0, i - k), min(n, i + k + 1)):
                grown[j] = True
        return rs, grown

    def gaps(self, grown):
        out, start = [], None
        for i, b in enumerate(grown + [True]):
            if not b and start is None:
                start = i
            elif b and start is not None:
                if i - start >= self.min_gap_slots:
                    out.append((start, i - 1))
                start = None
        return out

    # ------------------------------------------------------------------ the algorithm
    def step(self, p):
        front = p.sector_min(math.radians(-15), math.radians(15))
        if math.isnan(front):                      # never drive blind
            return Command(0.0, 0.0)

        goal = p.goal_bearing()
        rs, grown = self.blocked_map(p)
        gaps = self.gaps(grown)

        def angle(i):
            return p.angle_min + i * p.angle_increment

        best = None                                # (score, heading)
        for a, b in gaps:
            lo, hi = a, b
            if b - a >= 2 * self.edge_margin:      # keep away from the edges if the gap is wide enough
                lo, hi = a + self.edge_margin, b - self.edge_margin
            else:
                lo = hi = (a + b) // 2
            h = max(angle(lo), min(angle(hi), goal))    # the direction in the gap closest to the goal
            idx = int(round((h - p.angle_min) / p.angle_increment))
            depth = min(rs[max(0, idx - 1):idx + 2]) / self.look_max      # 0..1, how far you can see along it
            score = abs(h - goal) - self.depth_weight * depth
            if self.last_side and h * self.last_side < -0.05:
                score += self.side_bonus
            if best is None or score < best[0]:
                best = (score, h)

        if best is None:                           # no gap: turn on the spot towards the roomier side
            if self.spin_dir == 0:
                left = sum(rs[i] for i in range(len(rs)) if angle(i) > 0)
                right = sum(rs[i] for i in range(len(rs)) if angle(i) < 0)
                self.spin_dir = 1 if left >= right else -1
            cmd = Command(0.0, self.spin_dir * self.spin_speed)
            mode = 'no gap, spinning'
        else:
            heading = best[1]
            if abs(heading) > 0.05:
                self.last_side = 1 if heading > 0 else -1
            if self.spin_dir != 0 and abs(heading) > 0.6 * self.sharp:
                cmd = Command(0.0, self.spin_dir * self.spin_speed)      # keep spinning the way we started
            elif abs(heading) > self.sharp:
                self.spin_dir = 1 if heading > 0 else -1                 # start a spin and stick to it
                cmd = Command(0.0, self.spin_dir * self.spin_speed)
            else:
                self.spin_dir = 0
                room = max(0.0, min(1.0, (front - 0.20) / 0.40))          # slow down when the front is close
                align = math.cos(heading)
                speed = max(self.min_speed, self.cruise_speed * room * align)
                cmd = Command(speed, self.turn_gain * heading)
            mode = f'gap at {math.degrees(heading):+.0f} deg'

        if self.debug and p.time - self.last_print >= 1.0:
            self.last_print = p.time
            print(f'[follow_the_gap] t={p.time:.1f}s goal={math.degrees(goal):+.0f} deg gaps={len(gaps)} '
                  f'{mode} -> speed={cmd.linear:.2f} turn={cmd.angular:+.2f}', flush=True)
        return cmd
