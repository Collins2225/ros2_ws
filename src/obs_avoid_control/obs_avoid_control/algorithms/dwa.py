"""Algorithm 11: DWA, the Dynamic Window Approach (Fox, Burgard and Thrun, 1997). Our version for a 150-degree sonar.

Idea: do not choose a direction, choose a MOTION. The robot tries many pairs (speed, turn rate), works out for
each pair where it would end up, throws away the pairs that would hit something, scores the rest, and uses the best.

Steps (every 0.05 s):
  0. LOCAL MAP. The sonar sees only 150 degrees, so the robot keeps a small memory map of what it saw a moment
     ago (the same kind of grid as in VFH) and treats the sure squares as obstacle points. The 1997 paper also
     uses a local map.
  1. THE DYNAMIC WINDOW. The robot cannot change its speed instantly. From the speed and turn rate it is using
     now, only the pairs it can reach in one step are allowed (speed +- a_lin * dt, turn +- a_ang * dt), and never
     more than the limits 0.20 m/s and 1.0 rad/s.
  2. SAMPLE the window: `n_v` speeds x `n_w` turn rates.
  3. SAFETY. A pair means driving along a circle arc. The robot measures how far it could drive along that arc
     before it came closer than `robot_r` to an obstacle point (`free path`). A pair is thrown away if that
     is zero, or if the robot could not stop in time: speed must be at most sqrt(2 * free path * braking).
  4. SCORE the pairs that are left:   score = a * HEADING + b * CLEARANCE + c * SPEED
       HEADING   how well the robot points at the goal after 2 seconds on this pair (1 = at it, 0 = backwards),
       CLEARANCE the free path along the arc (capped at `clear_cap`), so arcs that run into things score low,
       SPEED     how fast the pair is (so the robot does not creep).
  5. Use the best pair. Turning on the spot (speed 0) is always allowed unless something is touching the robot.
  6. If NO pair is allowed (the robot is already too close to something), take the move that gets it
     farthest from the nearest obstacle point.
  7. RECOVERY (not in the paper): if the robot has been almost stopped for `stall_time` seconds, it turns on the
     spot to the roomier side until the way ahead is free, then DWA takes over again.
"""
import math
import os

from obs_avoid_control.core.interface import Algorithm, Command


class Controller(Algorithm):
    name = 'dwa'

    def __init__(self):
        self.v_max = 0.20              # m/s, the console's speed limit
        self.w_max = 1.0               # rad/s
        self.a_lin = 0.50              # m/s^2: how fast the speed may change
        self.a_ang = 5.0               # rad/s^2: how fast the turn rate may change
        self.n_v, self.n_w = 3, 9     # samples in the window
        self.horizon = float(os.environ.get('DWA_T', '2.0'))             # s: how far ahead each arc is predicted
        self.robot_r = float(os.environ.get('DWA_R', '0.19'))            # m: robot radius plus a margin
        self.spin_r = 0.16             # m: turning on the spot sweeps a circle of about this radius around the centre
        self.sonar_x = 0.10            # m: the sonar sits this far in front of the robot's centre
        self.look_max = 1.50           # m
        self.cell = 0.05               # m: size of one square of the local memory map
        self.hit_gain, self.miss_gain, self.cert_max = 3, 1, 15
        self.sure = 4                  # a square counts as an obstacle when its certainty is at least this
        self.clear_cap = float(os.environ.get('DWA_CAP', '0.80'))        # m: clearance above this scores the same
        self.w_head = float(os.environ.get('DWA_A', '1.0'))
        self.w_clear = float(os.environ.get('DWA_B', '1.5'))
        self.w_speed = float(os.environ.get('DWA_C', '0.4'))
        self.stall_time = float(os.environ.get('DWA_STALL', '1.0'))       # s: slower than `stall_speed` for this long = stuck
        self.stall_speed = 0.03        # m/s
        self.escape_free = 0.70        # m: the escape turn ends when the way ahead is free this far
        self.spin_speed = 0.7          # rad/s
        self.debug = os.environ.get('ALGO_DEBUG') == '1'
        self.reset()

    def reset(self):
        self.cur_v = 0.0               # the speed and turn we are using now (what we commanded last)
        self.cur_w = 0.0
        self.last_t = None
        self.last_print = -10.0
        self.grid = {}                 # local memory map: (ix, iy) -> certainty 0..15, in room coordinates
        self.slow_since = None         # time when the robot became (nearly) stopped
        self.escape_dir = 0            # 0 = not escaping, +1 / -1 = turning on the spot to the left / right

    def update_grid(self, p):
        c = self.cell
        ox, oy = p.x + self.sonar_x * math.cos(p.yaw), p.y + self.sonar_x * math.sin(p.yaw)    # the sonar sits in front of the centre
        for i, r in enumerate(p.ranges):
            if math.isnan(r):
                continue
            a = p.yaw + p.angle_min + i * p.angle_increment
            ca, sa = math.cos(a), math.sin(a)
            hit = r < self.look_max
            end = r if hit else self.look_max
            d = 0.05
            last = None
            while d < end:
                key = (int(math.floor((ox + d * ca) / c)), int(math.floor((oy + d * sa) / c)))
                if key != last:
                    v = self.grid.get(key, 0) - self.miss_gain
                    if v > 0:
                        self.grid[key] = v
                    else:
                        self.grid.pop(key, None)
                    last = key
                d += c * 0.5
            if hit:
                key = (int(math.floor((ox + r * ca) / c)), int(math.floor((oy + r * sa) / c)))
                self.grid[key] = min(self.cert_max, self.grid.get(key, 0) + self.hit_gain + self.miss_gain)
        if len(self.grid) > 6000:
            self.grid = {k: v for k, v in self.grid.items()
                         if math.hypot((k[0] + 0.5) * c - p.x, (k[1] + 0.5) * c - p.y) < 3.0}

    def obstacle_points(self, p, reach):
        """the sure squares of the memory map near the robot, in the robot's own frame (x ahead, y left), thinned to one per 8 cm"""
        c = self.cell
        cy, sy = math.cos(p.yaw), math.sin(p.yaw)
        seen = set()
        pts = []
        for (ix, iy), v in self.grid.items():
            if v < self.sure:
                continue
            dx, dy = (ix + 0.5) * c - p.x, (iy + 0.5) * c - p.y
            if dx * dx + dy * dy > reach * reach:
                continue
            fx, fy = cy * dx + sy * dy, -sy * dx + cy * dy
            key = (int(round(fx / 0.08)), int(round(fy / 0.08)))
            if key in seen:
                continue
            seen.add(key)
            pts.append((fx, fy))
        return pts

    def free_path(self, pts, curvature):
        """length of the arc (turn per metre = curvature) that the robot can drive before it comes closer than robot_r to a point"""
        ds = 0.05
        x = y = th = 0.0
        r2 = self.robot_r * self.robot_r
        s = 0.0
        while s < self.clear_cap:
            th += curvature * ds
            x += math.cos(th) * ds
            y += math.sin(th) * ds
            s += ds
            for (ox, oy) in pts:
                if (ox - x) * (ox - x) + (oy - y) * (oy - y) < r2:
                    return s - ds
        return self.clear_cap

    def step(self, p):
        front = p.sector_min(math.radians(-15), math.radians(15))
        if math.isnan(front):                          # never drive blind
            self.cur_v = self.cur_w = 0.0
            return Command(0.0, 0.0)

        dt = 0.05 if self.last_t is None else max(0.02, min(0.2, p.time - self.last_t))
        self.last_t = p.time

        # ---- the local memory map, and from it the obstacle points in the robot's own frame (x ahead, y left) ----
        self.update_grid(p)
        pts = self.obstacle_points(p, self.v_max * self.horizon + self.robot_r + 0.3)

        goal = p.goal_bearing()

        # ---- recovery: DWA only looks a couple of seconds ahead, so a box dead ahead of the goal can stop it for good.
        #      If the robot has been (nearly) stopped for `stall_time`, it turns on the spot to the roomier side until
        #      the way ahead is free, then DWA takes over again. (This rule is not part of the 1997 paper.) ----
        if self.escape_dir != 0:
            ahead = p.sector_min(math.radians(-25), math.radians(25))
            if not math.isnan(ahead) and ahead >= self.escape_free:
                self.escape_dir = 0
                self.slow_since = None
            else:
                self.cur_v, self.cur_w = 0.0, self.escape_dir * self.spin_speed
                cmd = Command(0.0, self.cur_w)
                self.print_debug(p, goal, 'stuck, turning on the spot', cmd)
                return cmd
        if self.cur_v < self.stall_speed and p.goal_distance() > 0.3:
            if self.slow_since is None:
                self.slow_since = p.time
            elif p.time - self.slow_since >= self.stall_time and not any(
                    ox * ox + oy * oy < self.spin_r * self.spin_r for (ox, oy) in self.obstacle_points(p, 0.3)):
                left = sum(r for i, r in enumerate(p.ranges) if not math.isnan(r) and p.angle_min + i * p.angle_increment > 0)
                right = sum(r for i, r in enumerate(p.ranges) if not math.isnan(r) and p.angle_min + i * p.angle_increment < 0)
                self.escape_dir = 1 if left >= right else -1
                return Command(0.0, self.escape_dir * self.spin_speed)
        else:
            self.slow_since = None

        # ---- 1. the dynamic window ----
        v_lo = max(0.0, self.cur_v - self.a_lin * dt)
        v_hi = min(self.v_max, self.cur_v + self.a_lin * dt)
        w_lo = max(-self.w_max, self.cur_w - self.a_ang * dt)
        w_hi = min(self.w_max, self.cur_w + self.a_ang * dt)
        vs = [v_lo + (v_hi - v_lo) * k / (self.n_v - 1) for k in range(self.n_v)]
        ws = [w_lo + (w_hi - w_lo) * k / (self.n_w - 1) for k in range(self.n_w)]
        spin_blocked = any(ox * ox + oy * oy < self.spin_r * self.spin_r for (ox, oy) in pts)
        cands = []                                     # (v, w, heading, clearance)
        for v in vs:
            for w in ws:
                # where would we point after `horizon` seconds on this pair?
                th = w * self.horizon
                heading = (math.pi - abs(math.atan2(math.sin(goal - th), math.cos(goal - th)))) / math.pi
                if v > 1e-6:
                    # how far can we drive along this very arc before we would touch something? (same turn per metre)
                    dist = self.free_path(pts, w / v)
                    if dist <= 0.0:
                        continue                         # would hit something at once
                    if v > math.sqrt(2.0 * dist * self.a_lin):
                        continue                         # could not stop in time
                    clearance = min(dist, self.clear_cap) / self.clear_cap
                else:
                    if abs(w) > 1e-6 and spin_blocked:
                        continue                         # something is too close to turn on the spot
                    clearance = 0.0                      # a spin on the spot is safe but gives no clearance score
                cands.append((v, w, heading, clearance))

        if not cands:
            # nothing is allowed (we are already closer to something than we like): take the move of the window that
            # gets us farthest from the nearest obstacle point in the next half second
            best_escape = None
            for v in vs:
                for w in ws:
                    x = y = th = 0.0
                    nearest = math.inf
                    for _ in range(5):
                        th += w * 0.1
                        x += v * math.cos(th) * 0.1
                        y += v * math.sin(th) * 0.1
                        for (ox, oy) in pts:
                            d = math.hypot(ox - x, oy - y)
                            if d < nearest:
                                nearest = d
                    key = (round(nearest, 2), v, -abs(math.atan2(math.sin(goal - th), math.cos(goal - th))))
                    if best_escape is None or key > best_escape[0]:
                        best_escape = (key, v, w)
            _, v, w = best_escape
            self.cur_v, self.cur_w = v, w
            self.print_debug(p, goal, 'nothing allowed, moving away from the nearest obstacle', Command(v, w))
            return Command(v, w)

        best = None
        for v, w, heading, clearance in cands:
            score = self.w_head * heading + self.w_clear * clearance + self.w_speed * (v / self.v_max)
            if best is None or score > best[0]:
                best = (score, v, w)
        _, v, w = best
        self.cur_v, self.cur_w = v, w
        cmd = Command(v, w)
        self.print_debug(p, goal, f'{len(cands)} pairs allowed', cmd)
        return cmd

    def print_debug(self, p, goal, mode, cmd):
        if self.debug and p.time - self.last_print >= 1.0:
            self.last_print = p.time
            print(f'[dwa] t={p.time:.1f}s goal={math.degrees(goal):+.0f} deg {mode} '
                  f'-> speed={cmd.linear:.2f} turn={cmd.angular:+.2f}', flush=True)
