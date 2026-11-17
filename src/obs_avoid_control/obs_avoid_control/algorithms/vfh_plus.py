"""Algorithm 8: VFH+ (Ulrich and Borenstein, 1998), the improved Vector Field Histogram.

VFH+ keeps the memory map and the "crowdedness per direction" histogram of VFH, and fixes four weak spots:
  1. Robot size: every square is made fatter by the robot's radius, so the histogram already says which
     directions the ROBOT (not a point) can really use. No extra smoothing is needed.
  2. Two thresholds (hysteresis): a direction becomes BLOCKED only when it is more crowded than the HIGH
     threshold, and becomes FREE again only when it is calmer than the LOW threshold. In between it keeps
     its old state. This stops the "free, blocked, free, blocked" flicker that makes VFH dither.
  3. The robot cannot turn on a point while driving: it needs a turning circle. Directions that lie behind
     an obstacle which the turning circle would hit are MASKED (blocked) as well.
  4. Choosing a direction: every candidate direction gets a COST. Cost = 5 x (how far from the goal)
     + 2 x (how far from where the robot is facing now) + 2 x (how far from the direction chosen last time).
     The cheapest candidate wins. The last two terms make the robot decisive and smooth.

Steps (every 0.05 s):
  1. Update the memory map from the sonar picture (as in VFH).
  2. Build the polar histogram in ROOM directions (72 sectors of 5 degrees) from the squares inside the window.
     strength of a square = (certainty/15)^2 * (A - B * distance^2)  (near squares count a lot).
  3. Make the binary histogram with the two thresholds (keeps its state from the last step).
  4. Mask the sectors the turning circle cannot reach.
  5. Find the VALLEYS (runs of free sectors). Candidate directions: the goal direction if it is inside a valley;
     in a narrow valley its middle; in a wide valley a point `wide / 2` sectors inside each edge.
  6. Pick the candidate with the lowest cost, steer to it; slow down when crowded or when a big turn is needed.
  7. If there is no valley: stop and turn on the spot towards the calmer side.
"""
import math
import os

from obs_avoid_control.core.interface import Algorithm, Command


class Controller(Algorithm):
    name = 'vfh_plus'

    def __init__(self):
        # --- the memory map (same as VFH) ---
        self.cell = 0.05
        self.window = float(os.environ.get('VFHP_WINDOW', '1.00'))
        self.hit_gain, self.miss_gain, self.cert_max = 3, 1, 15
        self.start_clear = 0.10
        # --- the polar histogram ---
        self.n_sectors = 72
        self.robot_r = float(os.environ.get('VFHP_R', '0.14'))        # m: robot radius plus a safety margin
        self.mag_a = 3.0                                              # strength of a sure square right at the robot
        self.mag_far = 0.3                                            # strength of a sure square at the window edge
        self.t_low = float(os.environ.get('VFHP_TLOW', '0.45'))       # below this a direction becomes free
        self.t_high = float(os.environ.get('VFHP_THIGH', '0.90'))     # above this a direction becomes blocked
        self.turn_r = float(os.environ.get('VFHP_RTURN', '0.20'))     # m: turning-circle radius (0.20 m/s at 1.0 rad/s)
        self.wide = int(os.environ.get('VFHP_WIDE', '10'))            # sectors
        # --- the cost function ---
        self.mu_goal = float(os.environ.get('VFHP_MU1', '5'))
        self.mu_head = float(os.environ.get('VFHP_MU2', '2'))
        self.mu_prev = float(os.environ.get('VFHP_MU3', '2'))
        # --- driving ---
        self.cruise_speed = 0.20
        self.min_speed = 0.04
        self.crowd_max = 0.9
        self.turn_gain = 1.6
        self.spin_speed = 0.7
        self.sharp = math.radians(55)
        self.debug = os.environ.get('ALGO_DEBUG') == '1'
        self.reset()

    def reset(self):
        self.grid = {}
        self.blocked = [False] * self.n_sectors      # binary histogram of the last step (for the hysteresis)
        self.last_print = -10.0
        self.spin_dir = 0
        self.prev_k = None                           # sector chosen last time (room directions)

    # ------------------------------------------------------------------ step 1: the memory map
    def update_grid(self, p):
        c = self.cell
        for i, r in enumerate(p.ranges):
            if math.isnan(r):
                continue
            a = p.yaw + p.angle_min + i * p.angle_increment
            ca, sa = math.cos(a), math.sin(a)
            hit = r < self.window
            end = r if hit else self.window
            d = self.start_clear
            last = None
            while d < end:
                key = (int(math.floor((p.x + d * ca) / c)), int(math.floor((p.y + d * sa) / c)))
                if key != last:
                    v = self.grid.get(key, 0) - self.miss_gain
                    if v > 0:
                        self.grid[key] = v
                    else:
                        self.grid.pop(key, None)
                    last = key
                d += c * 0.5
            if hit:
                key = (int(math.floor((p.x + r * ca) / c)), int(math.floor((p.y + r * sa) / c)))
                self.grid[key] = min(self.cert_max, self.grid.get(key, 0) + self.hit_gain + self.miss_gain)
        if len(self.grid) > 6000:
            keep = {}
            for (ix, iy), v in self.grid.items():
                if math.hypot((ix + 0.5) * c - p.x, (iy + 0.5) * c - p.y) < 2.5 * self.window:
                    keep[(ix, iy)] = v
            self.grid = keep

    # ------------------------------------------------------------------ helpers
    def sector_of(self, a):
        """room direction (rad) -> sector index"""
        width = 2 * math.pi / self.n_sectors
        return int((math.atan2(math.sin(a), math.cos(a)) + math.pi) / width) % self.n_sectors

    def angle_of(self, k):
        """sector index -> room direction (rad, -pi..pi)"""
        width = 2 * math.pi / self.n_sectors
        return -math.pi + (k + 0.5) * width

    def dist(self, a, b):
        d = abs(a - b) % self.n_sectors
        return min(d, self.n_sectors - d)

    # ------------------------------------------------------------------ steps 2-4: the histograms
    def histograms(self, p, window):
        n = self.n_sectors
        width = 2 * math.pi / n
        c = self.cell
        h = [0.0] * n
        b_coef = (self.mag_a - self.mag_far) / (window * window)
        cells = []                                   # (distance, room angle) of every sure square in the window
        for (ix, iy), v in self.grid.items():
            dx, dy = (ix + 0.5) * c - p.x, (iy + 0.5) * c - p.y
            d = math.hypot(dx, dy)
            if d >= window or d < 1e-6:
                continue
            beta = math.atan2(dy, dx)
            cells.append((d, beta, dx, dy))
            k = self.sector_of(beta)
            strength = (v / self.cert_max) ** 2 * (self.mag_a - b_coef * d * d)
            spread = int(math.ceil(math.asin(min(1.0, self.robot_r / d)) / width)) if self.robot_r > 0 else 0
            for j in range(-spread, spread + 1):
                h[(k + j) % n] += strength
        # binary histogram with two thresholds (hysteresis)
        binary = []
        for k in range(n):
            if h[k] > self.t_high:
                binary.append(True)
            elif h[k] < self.t_low:
                binary.append(False)
            else:
                binary.append(self.blocked[k])
        self.blocked = binary
        masked = list(binary)
        # masking: directions the turning circle cannot reach
        if self.turn_r > 0:
            cy, sy = math.cos(p.yaw), math.sin(p.yaw)
            lim = self.turn_r + self.robot_r
            left_lim, right_lim = None, None         # smallest / largest angle (robot frame) of a square inside a circle
            for d, beta, dx, dy in cells:
                fx, fy = cy * dx + sy * dy, -sy * dx + cy * dy        # robot frame: x ahead, y left
                rel = math.atan2(fy, fx)
                if math.hypot(fx, fy - self.turn_r) < lim and fy > 0:   # inside the left turning circle
                    left_lim = rel if left_lim is None else min(left_lim, rel)
                if math.hypot(fx, fy + self.turn_r) < lim and fy < 0:   # inside the right turning circle
                    right_lim = rel if right_lim is None else max(right_lim, rel)
            for k in range(n):
                rel = math.atan2(math.sin(self.angle_of(k) - p.yaw), math.cos(self.angle_of(k) - p.yaw))
                if left_lim is not None and rel > left_lim:
                    masked[k] = True
                if right_lim is not None and rel < right_lim:
                    masked[k] = True
        return h, masked

    # ------------------------------------------------------------------ the algorithm
    def step(self, p):
        front = p.sector_min(math.radians(-15), math.radians(15))
        if math.isnan(front):
            return Command(0.0, 0.0)

        self.update_grid(p)
        window = max(0.45, min(self.window, p.goal_distance() + 0.30))
        h, masked = self.histograms(p, window)
        n = self.n_sectors
        free = [not b for b in masked]
        goal_room = p.yaw + p.goal_bearing()
        kt = self.sector_of(goal_room)
        kh = self.sector_of(p.yaw)

        # valleys: runs of free sectors (the circle wraps around)
        valleys = []
        if all(free):
            valleys.append((0, n - 1, True))
        elif any(free):
            s0 = next(k for k in range(n) if not free[k])
            run = None
            for off in range(1, n + 1):
                k = (s0 + off) % n
                if free[k]:
                    run = [k, k] if run is None else [run[0], k]
                elif run is not None:
                    valleys.append((run[0], run[1], False))
                    run = None

        def length(v):
            a, b, whole = v
            return n if whole else ((b - a) % n) + 1

        def inside(v, k):
            a, b, whole = v
            return whole or ((k - a) % n) <= ((b - a) % n)

        if not valleys:
            if self.spin_dir == 0:
                left = sum(h[k] for k in range(n) if math.sin(self.angle_of(k) - p.yaw) > 0)
                right = sum(h[k] for k in range(n) if math.sin(self.angle_of(k) - p.yaw) < 0)
                self.spin_dir = 1 if left <= right else -1
            cmd = Command(0.0, self.spin_dir * self.spin_speed)
            self.print_debug(p, goal_room, 0, 'no valley, spinning', cmd)
            return cmd

        # candidate directions
        cands = []                                   # sector indices
        for v in valleys:
            a, b, whole = v
            if whole:
                cands.append(kt)
                continue
            L = length(v)
            if L <= self.wide:
                cands.append((a + (L - 1) // 2) % n)
            else:
                half = self.wide // 2
                cr, cl = (a + half) % n, (b - half) % n
                cands.append(cr)
                cands.append(cl)
                if inside(v, kt) and self.dist(kt, a) >= half and self.dist(kt, b) >= half:
                    cands.append(kt)
                elif inside(v, kt):
                    cands.append(kt)                 # the goal is near an edge: still a candidate
        best = None
        for k in cands:
            prev = 0 if self.prev_k is None else self.dist(k, self.prev_k)
            g = self.mu_goal * self.dist(k, kt) + self.mu_head * self.dist(k, kh) + self.mu_prev * prev
            if best is None or g < best[0]:
                best = (g, k)
        k = best[1]
        self.prev_k = k
        if k == kt:
            heading = p.goal_bearing()               # aim at the goal itself, not at the middle of its sector
        else:
            heading = math.atan2(math.sin(self.angle_of(k) - p.yaw), math.cos(self.angle_of(k) - p.yaw))

        if self.spin_dir != 0 and abs(heading) > 0.6 * self.sharp:
            if abs(heading) < math.radians(150):
                self.spin_dir = 1 if heading > 0 else -1
            cmd = Command(0.0, self.spin_dir * self.spin_speed)
            mode = 'keep spinning'
        elif abs(heading) > self.sharp:
            self.spin_dir = 1 if heading > 0 else -1
            cmd = Command(0.0, self.spin_dir * self.spin_speed)
            mode = 'turn on the spot'
        else:
            self.spin_dir = 0
            crowd = max(h[(k + j) % n] for j in (-1, 0, 1))
            speed = self.cruise_speed * (1.0 - min(crowd, self.crowd_max) / self.crowd_max)
            speed *= max(0.0, math.cos(heading))
            speed = max(self.min_speed, speed)
            cmd = Command(speed, self.turn_gain * heading)
            mode = f'valley, steer {math.degrees(heading):+.0f} deg'
        self.print_debug(p, goal_room, len(valleys), mode, cmd)
        return cmd

    def print_debug(self, p, goal_room, nvalleys, mode, cmd):
        if self.debug and p.time - self.last_print >= 1.0:
            self.last_print = p.time
            print(f'[vfh_plus] t={p.time:.1f}s valleys={nvalleys} cells={len(self.grid)} '
                  f'{mode} -> speed={cmd.linear:.2f} turn={cmd.angular:+.2f}', flush=True)
