"""Algorithm 7: VFH, the Vector Field Histogram (Borenstein and Koren, 1991).

Idea: the robot keeps a MEMORY MAP of the room (a grid of small squares, each with a "how sure am I
that something is here" number). From the map it builds a histogram of how crowded every direction
is, and then steers into a calm valley of that histogram that points closest to the goal.

Steps (every 0.05 s):
  1. Update the memory map from the sonar picture. For every direction that was looked at:
       - the squares the beam passed through become a little less sure (-1),
       - the square where the beam stopped becomes more sure (+3). Numbers stay between 0 and 15.
     The map is kept in room coordinates, so the robot still remembers what it saw a moment ago.
  2. Look only at the squares inside a round WINDOW around the robot (radius `window`).
  3. Build the POLAR HISTOGRAM: cut the full circle into 72 sectors of 5 degrees. Every sure square
     adds an "obstacle strength" to its sector:  strength = (certainty/15)^2 * (1 - distance/window).
     Near and sure squares count a lot, far and unsure ones almost nothing.
  4. Smooth the histogram a little (neighbouring sectors share their strength).
  5. A sector is FREE if its smoothed strength is below the threshold `threshold`.
     A VALLEY is a run of neighbouring free sectors.
  6. Choose the valley that has a sector closest to the goal direction. Inside the valley:
       - if the goal direction is in the valley, steer straight at the goal,
       - in a wide valley (more than `wide` sectors), steer `wide / 2` sectors inside from the edge nearest the goal,
       - in a narrow valley, steer at its middle.
  7. Speed: slower when the chosen direction is crowded, and slower when a big turn is needed.
     A very big turn is done on the spot.
  8. If no valley exists at all: stop and turn on the spot towards the calmest side.
"""
import math
import os

from obs_avoid_control.core.interface import Algorithm, Command


class Controller(Algorithm):
    name = 'vfh'

    def __init__(self):
        # --- the memory map ---
        self.cell = 0.05               # m: size of one square
        self.window = float(os.environ.get('VFH_WINDOW', '1.00'))   # m: radius of the window around the robot that is used
        self.hit_gain, self.miss_gain, self.cert_max = 3, 1, 15
        self.start_clear = 0.10        # m: the beam is assumed to start this far from the robot's centre
        # --- the polar histogram ---
        self.n_sectors = 72            # 5 degrees each
        self.robot_r = float(os.environ.get('VFH_R', '0.14'))      # m: robot radius plus a safety margin (obstacles are made fatter by this)
        self.smooth = int(os.environ.get('VFH_SMOOTH', '2'))       # sectors on each side that share their strength
        self.threshold = float(os.environ.get('VFH_T', '0.60'))    # a sector below this is free
        self.wide = int(os.environ.get('VFH_WIDE', '10'))          # sectors: valleys wider than this are "wide"
        # --- driving ---
        self.cruise_speed = 0.20       # m/s, the console's speed limit
        self.min_speed = 0.04          # m/s
        self.crowd_max = 0.6           # crowdedness at which the speed drops to the minimum
        self.turn_gain = 1.6           # rad/s per radian of heading error
        self.spin_speed = 0.7          # rad/s when turning on the spot
        self.sharp = math.radians(55)  # heading errors above this: turn on the spot
        self.stick = 4                 # sectors of bonus for staying with the valley chosen last time
        self.tie_break = int(os.environ.get('VFH_TIE', '1'))       # sectors: on a tie, keep going round the side chosen last time
        self.debug = os.environ.get('ALGO_DEBUG') == '1'
        self.reset()

    def reset(self):
        self.grid = {}                 # (ix, iy) -> certainty 0..15
        self.last_print = -10.0
        self.spin_dir = 0
        self.last_world_dir = None     # last steering direction, in room coordinates (rad)

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
        # forget squares far away, so the map stays small
        if len(self.grid) > 6000:
            keep = {}
            for (ix, iy), v in self.grid.items():
                if math.hypot((ix + 0.5) * c - p.x, (iy + 0.5) * c - p.y) < 2.5 * self.window:
                    keep[(ix, iy)] = v
            self.grid = keep

    # ------------------------------------------------------------------ steps 2-4: the histogram
    def histogram(self, p, window):
        n = self.n_sectors
        width = 2 * math.pi / n
        h = [0.0] * n
        c = self.cell
        for (ix, iy), v in self.grid.items():
            dx, dy = (ix + 0.5) * c - p.x, (iy + 0.5) * c - p.y
            d = math.hypot(dx, dy)
            if d >= window or d < 1e-6:
                continue
            beta = math.atan2(dy, dx) - p.yaw
            beta = math.atan2(math.sin(beta), math.cos(beta))          # -pi..pi, 0 = straight ahead
            k = int((beta + math.pi) / width) % n
            strength = (v / self.cert_max) ** 2 * (1.0 - d / window)
            # the robot is not a point: the square also blocks the sectors it would hit if the robot drove past it
            spread = int(math.ceil(math.asin(min(1.0, self.robot_r / d)) / width)) if self.robot_r > 0 else 0
            for j in range(-spread, spread + 1):
                h[(k + j) % n] += strength
        l = self.smooth
        sm = []
        for k in range(n):
            tot = 0.0
            for j in range(-l, l + 1):
                tot += (l - abs(j) + 1) * h[(k + j) % n]
            sm.append(tot / (2 * l + 1))
        return sm

    # sector index <-> angle (robot frame, 0 = straight ahead, left positive)
    def sector_of(self, beta):
        width = 2 * math.pi / self.n_sectors
        return int((beta + math.pi) / width) % self.n_sectors

    def angle_of(self, k):
        width = 2 * math.pi / self.n_sectors
        return -math.pi + (k + 0.5) * width

    # ------------------------------------------------------------------ the algorithm
    def step(self, p):
        front = p.sector_min(math.radians(-15), math.radians(15))
        if math.isnan(front):                      # never drive blind
            return Command(0.0, 0.0)

        self.update_grid(p)
        # things behind the goal cannot stop the robot from arriving, so the window never reaches far past the goal
        window = max(0.45, min(self.window, p.goal_distance() + 0.30))
        hist = self.histogram(p, window)
        n = self.n_sectors
        free = [v < self.threshold for v in hist]
        goal = p.goal_bearing()
        kt = self.sector_of(goal)

        # valleys: runs of free sectors (the circle wraps around)
        valleys = []
        if all(free):
            valleys.append((0, n - 1, True))
        elif any(free):
            s0 = next(k for k in range(n) if not free[k])             # begin at a blocked sector
            run = None
            for off in range(1, n + 1):
                k = (s0 + off) % n
                if free[k]:
                    run = [k, k] if run is None else [run[0], k]
                elif run is not None:
                    valleys.append((run[0], run[1], False))
                    run = None

        def dist(a, b):
            d = abs(a - b) % n
            return min(d, n - d)

        def length(v):
            a, b, whole = v
            return n if whole else ((b - a) % n) + 1

        def inside(v, k):
            a, b, whole = v
            return whole or ((k - a) % n) <= ((b - a) % n)

        if not valleys:                            # everything is crowded: turn on the spot to the calmest side
            if self.spin_dir == 0:
                left = sum(hist[k] for k in range(n) if self.angle_of(k) > 0)
                right = sum(hist[k] for k in range(n) if self.angle_of(k) < 0)
                self.spin_dir = 1 if left <= right else -1
            cmd = Command(0.0, self.spin_dir * self.spin_speed)
            mode = 'no valley, spinning'
            self.print_debug(p, goal, hist, len(valleys), mode, cmd)
            return cmd

        # choose the valley that has a sector closest to the goal direction
        kl = None if self.last_world_dir is None else self.sector_of(
            math.atan2(math.sin(self.last_world_dir - p.yaw), math.cos(self.last_world_dir - p.yaw)))
        best = None
        for v in valleys:
            if inside(v, kt):
                near = kt
                gap = 0
            else:
                a, b, _ = v
                # the border nearest the goal; on a tie (or nearly) keep the side we were already going round
                cost_a = dist(a, kt) + (0 if kl is None or dist(a, kl) <= dist(b, kl) else self.tie_break)
                cost_b = dist(b, kt) + (0 if kl is None or dist(b, kl) < dist(a, kl) else self.tie_break)
                near = a if cost_a <= cost_b else b
                gap = dist(near, kt)
            score = gap - (self.stick if kl is not None and inside(v, kl) else 0)
            if best is None or score < best[0]:
                best = (score, v, near)
        _, v, near = best
        a, b, whole = v

        # where to steer inside the valley
        if inside(v, kt):
            target = kt
        elif length(v) <= self.wide:
            target = (a + (length(v) - 1) // 2) % n                    # narrow: the middle
        else:
            half = self.wide // 2
            target = (near + half) % n if near == a else (near - half) % n   # wide: a bit inside from the edge
        heading = goal if inside(v, kt) else self.angle_of(target)       # aiming at the goal itself needs no rounding to a sector
        self.last_world_dir = p.yaw + heading

        if self.spin_dir != 0 and abs(heading) > 0.6 * self.sharp:
            # keep turning, by the short way round; only when the target is almost straight behind keep the old direction
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
            crowd = max(hist[(target + j) % n] for j in (-1, 0, 1))
            speed = self.cruise_speed * (1.0 - min(crowd, self.crowd_max) / self.crowd_max)
            speed *= max(0.0, math.cos(heading))
            speed = max(self.min_speed, speed)
            cmd = Command(speed, self.turn_gain * heading)
            mode = f'valley, steer {math.degrees(heading):+.0f} deg'
        self.print_debug(p, goal, hist, len(valleys), mode, cmd)
        return cmd

    def print_debug(self, p, goal, hist, nvalleys, mode, cmd):
        if self.debug and p.time - self.last_print >= 1.0:
            self.last_print = p.time
            print(f'[vfh] t={p.time:.1f}s goal={math.degrees(goal):+.0f} deg valleys={nvalleys} cells={len(self.grid)} '
                  f'{mode} -> speed={cmd.linear:.2f} turn={cmd.angular:+.2f}', flush=True)
