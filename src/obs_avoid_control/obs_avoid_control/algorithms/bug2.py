"""Algorithm 9: Bug2 (Lumelsky and Stepanov, 1987), "follow the wall, leave it on the straight line".

Idea: draw an imaginary straight line from the START to the GOAL (the "m-line"). Drive along it. When an
obstacle is in the way, walk around it by keeping a wall at a steady distance, and leave the wall the moment
you are back ON the m-line AND closer to the goal than the place where you first hit the obstacle.

Two modes (every 0.05 s):
  GO      drive towards the goal (steer on the goal direction, 0.20 m/s).
          If something is closer than `hit_dist` straight ahead: remember this HIT POINT and its distance to
          the goal, choose which side to keep the wall on, and switch to FOLLOW.
  FOLLOW  keep the wall on the chosen side at `follow_dist`:
            - too far from the wall: turn towards it, too close: turn away (steering is proportional);
            - something ahead (closer than `corner_dist`): stop and turn away from the wall (an inside corner);
            - no wall seen at the side any more: curve towards the side to go round the corner (outside corner).
          Leave the wall (back to GO) when ALL of these are true:
            - the robot is within `line_tol` of the m-line,
            - it is closer to the goal than the hit point by at least `progress`,
            - the way towards the goal is free for `clear_dist`.
Bug2 has no memory map: it needs only its own position (to know where the m-line is) and the sonar.
"""
import math
import os

from obs_avoid_control.core.interface import Algorithm, Command


class Controller(Algorithm):
    name = 'bug2'

    def __init__(self):
        self.cruise_speed = 0.20       # m/s
        self.follow_speed = 0.15       # m/s while following a wall
        self.hit_dist = float(os.environ.get('BUG2_HIT', '0.40'))        # m: closer than this ahead = obstacle hit
        self.follow_dist = float(os.environ.get('BUG2_FOLLOW', '0.30'))  # m: wanted distance to the wall
        self.corner_dist = 0.35        # m: something this close ahead while following = inside corner
        self.line_tol = float(os.environ.get('BUG2_TOL', '0.12'))        # m: "on the m-line"
        self.progress = 0.15           # m: must be this much closer to the goal than the hit point
        self.clear_dist = 0.60         # m: the way to the goal must be free this far to leave the wall
        self.side_mode = os.environ.get('BUG2_SIDE', 'room')             # 'left', 'right' or 'room'
        self.turn_gain = 1.0
        self.wall_gain = 2.0           # rad/s per metre of distance error
        self.spin_speed = 0.7
        self.view_limit = math.radians(70)   # the goal must be this close to straight ahead to be checked with the sonar
        self.debug = os.environ.get('ALGO_DEBUG') == '1'
        self.reset()

    def reset(self):
        self.start = None
        self.mode = 'go'
        self.side = 1                  # +1 = wall on the left, -1 = wall on the right
        self.hit = None                # (x, y, distance to goal) where the wall was met
        self.spin_dir = 0
        self.refused = None            # place where leaving the wall was tried and the way was not free
        self.last_print = -10.0

    # ------------------------------------------------------------------ helpers
    @staticmethod
    def clamp(v, lo, hi):
        return max(lo, min(hi, v))

    def line_offset(self, p):
        """distance of the robot from the m-line (start -> goal)"""
        sx, sy = self.start
        dx, dy = p.goal_x - sx, p.goal_y - sy
        L = math.hypot(dx, dy)
        if L < 1e-6:
            return 0.0
        return abs(dx * (p.y - sy) - dy * (p.x - sx)) / L

    def room_side(self, p):
        """which side has more room: +1 left, -1 right (unknown readings count as 0)"""
        left = right = 0.0
        for i, r in enumerate(p.ranges):
            a = p.angle_min + i * p.angle_increment
            v = 0.0 if math.isnan(r) else min(r, 1.5)
            if a > math.radians(15):
                left += v
            elif a < -math.radians(15):
                right += v
        return 1 if left >= right else -1

    # ------------------------------------------------------------------ the algorithm
    def step(self, p):
        front = p.sector_min(math.radians(-15), math.radians(15))
        if math.isnan(front):                      # never drive blind
            return Command(0.0, 0.0)
        if self.start is None:
            self.start = (p.x, p.y)
        goal = p.goal_bearing()
        gd = p.goal_distance()

        if self.mode == 'go':
            # something in the way, ahead of us or on the way to the goal?
            toward = p.sector_min(goal - math.radians(15), goal + math.radians(15))
            blocked = front < self.hit_dist or (not math.isnan(toward) and abs(goal) < math.radians(70) and toward < self.hit_dist)
            if blocked:
                self.mode = 'follow'
                self.hit = (p.x, p.y, gd)
                self.refused = None
                if self.side_mode == 'left':
                    self.side = 1
                elif self.side_mode == 'right':
                    self.side = -1
                else:
                    self.side = self.room_side(p)
                self.spin_dir = 0
            else:
                turn = self.clamp(self.turn_gain * goal, -0.8, 0.8)
                cmd = Command(self.cruise_speed, turn)
                self.print_debug(p, 'go', cmd)
                return cmd

        # ---- FOLLOW ----
        s = self.side
        # leave the wall?  (on the m-line, closer to the goal than the hit point, and the way is free)
        on_line = self.line_offset(p) < self.line_tol and gd < self.hit[2] - self.progress
        if on_line and (self.refused is None or math.hypot(p.x - self.refused[0], p.y - self.refused[1]) > 0.30):
            if abs(goal) > self.view_limit:
                # the goal is outside what the sonar can see: turn on the spot until it is in view
                self.spin_dir = 1 if goal > 0 else -1
                cmd = Command(0.0, self.spin_dir * self.spin_speed)
                self.print_debug(p, 'on the m-line, turning to the goal', cmd)
                return cmd
            ahead = p.sector_min(goal - math.radians(15), goal + math.radians(15))
            if not math.isnan(ahead) and ahead > self.clear_dist and front > self.clear_dist:
                self.mode = 'go'
                self.refused = None
                self.print_debug(p, 'leave the wall', Command(0, 0))
                return Command(self.cruise_speed * 0.5, self.clamp(self.turn_gain * goal, -0.8, 0.8))
            self.refused = (p.x, p.y)                # not free here: do not ask again for the next 0.30 m

        # distance to the wall on our side (readings between 35 and 75 degrees), and the diagonal ahead
        side_vals, diag_vals = [], []
        for i, r in enumerate(p.ranges):
            a = s * (p.angle_min + i * p.angle_increment)       # angle measured towards the wall side
            if math.isnan(r):
                continue
            if math.radians(35) <= a <= math.radians(75):
                side_vals.append(r)
            elif math.radians(10) <= a < math.radians(35):
                diag_vals.append(r)
        d_side = min(side_vals) if side_vals else math.inf
        d_diag = min(diag_vals) if diag_vals else math.inf
        # a wall seen at 35-75 degrees is at (reading * sin(angle)) from the robot's side: use ~ reading * 0.85
        d_wall = min(d_side * 0.85, d_diag * 0.55)

        if front < self.corner_dist:
            # inside corner: turn away from the wall on the spot
            self.spin_dir = -s
            cmd = Command(0.0, self.spin_dir * self.spin_speed)
            mode = 'corner, turn away'
        elif d_wall == math.inf or d_wall > 1.0:
            # lost the wall: curve round the corner towards it
            cmd = Command(0.10, s * 0.6)
            mode = 'wall lost, curve round'
        else:
            err = d_wall - self.follow_dist           # positive: too far from the wall
            ang = self.clamp(s * self.wall_gain * err, -0.8, 0.8)
            slow = self.follow_speed * self.clamp((front - 0.25) / 0.35, 0.3, 1.0)
            cmd = Command(slow, ang)
            mode = f'wall at {d_wall:.2f} m'
        self.print_debug(p, 'follow ' + ('left' if s > 0 else 'right') + ': ' + mode, cmd)
        return cmd

    def print_debug(self, p, mode, cmd):
        if self.debug and p.time - self.last_print >= 1.0:
            self.last_print = p.time
            off = self.line_offset(p) if self.start else 0.0
            print(f'[bug2] t={p.time:.1f}s {mode} off-line={off:.2f} m goal={p.goal_distance():.2f} m '
                  f'-> speed={cmd.linear:.2f} turn={cmd.angular:+.2f}', flush=True)
