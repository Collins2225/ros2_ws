"""Algorithm 10: Nearness Diagram (ND), after Minguez and Montano (2004). Our version, built for a sonar that sees 150 degrees.

Idea: look at the sonar picture and find the GAPS between obstacles (a gap starts and ends where the distance
suddenly jumps). Pick the gap that is closest to the goal: the FREE WALKING AREA. Then decide HOW to drive by
asking two questions: "is any obstacle inside my SAFETY ZONE?" and "is the goal inside the free walking area?".
The answers put the robot into one of five SITUATIONS, and each situation has its own rule for steering:

  1. HSGR  (High Safety, Goal in Region)    nothing close, the goal is in the gap   -> drive straight at the goal.
  2. HSWR  (High Safety, Wide Region)       nothing close, the goal is outside a wide gap -> run along the gap's edge
                                            nearest the goal, a bit inside so we miss the obstacle.
  3. HSNR  (High Safety, Narrow Region)     nothing close, the gap is narrow        -> go through the middle of it.
  4. LS1   (Low Safety, one side)           an obstacle inside the safety zone on ONE side -> steer away from it
                                            more strongly the closer it is.
  5. LS2   (Low Safety, both sides)         obstacles inside the safety zone on BOTH sides -> creep through the middle,
                                            towards the side that has more room.

Steps (every 0.05 s):
  1. Read the sonar picture (one distance per 5 degrees, -75 to +75; readings are cut off at `look_max`).
     Past the edge of the picture we pretend there is a wall, so the robot never aims at something it cannot see.
  2. Find the DISCONTINUITIES: neighbouring readings that differ by more than `jump`. A jump up starts a gap,
     the next jump down ends it. Throw away gaps that are narrower than `min_width` (the robot does not fit).
  3. The free walking area is the gap that contains the goal direction, otherwise the gap whose edge is nearest
     the goal. A small bonus for the side chosen last time stops left-right dithering.
  4. Check the safety zone (`safety` metres around the robot) on the left and on the right.
  5. Pick the situation, steer by its rule, drive slower when the front is close, turn on the spot when the
     chosen direction is far off to one side. If there is no gap at all: turn on the spot to the roomier side.
"""
import math
import os

from obs_avoid_control.core.interface import Algorithm, Command


class Controller(Algorithm):
    name = 'nearness_diagram'

    def __init__(self):
        self.cruise_speed = 0.20       # m/s
        self.min_speed = 0.04          # m/s
        self.look_max = 1.50           # m: readings are cut off here
        self.jump = float(os.environ.get('ND_JUMP', '0.35'))             # m: a change between neighbours bigger than this is a discontinuity
        self.min_width = float(os.environ.get('ND_WIDTH', '0.40'))       # m: a gap narrower than this is ignored
        self.safety = float(os.environ.get('ND_SAFETY', '0.35'))         # m: the safety zone around the robot
        self.open_depth = float(os.environ.get('ND_DEPTH', '0.65'))      # m: some direction in a gap must see at least this far
        self.clear = 0.20              # m: robot radius + margin, used to steer clear of a gap edge
        self.wide_angle = math.radians(float(os.environ.get('ND_WIDE', '60')))   # a gap wider than this is "wide"
        self.turn_gain = 1.6           # rad/s per radian of heading error
        self.spin_speed = 0.7          # rad/s when turning on the spot
        self.sharp = math.radians(55)  # heading errors above this: turn on the spot
        self.side_bonus = 0.25         # rad: penalty for switching to the other side
        self.debug = os.environ.get('ALGO_DEBUG') == '1'
        self.reset()

    def reset(self):
        self.last_print = -10.0
        self.last_side = 0
        self.spin_dir = 0

    # ------------------------------------------------------------------ steps 1-3: find the gaps
    def find_regions(self, rs, ang, n, depth_needed):
        """Return gaps as (left_index, right_index): free space between the obstacle at left_index and the one at right_index."""
        # pretend there is a wall just outside the picture
        ext = [0.0] + rs + [0.0]
        regions = []
        start = None                                  # index in ext of the obstacle where the open gap begins
        for i in range(len(ext) - 1):
            step = ext[i + 1] - ext[i]
            if step > self.jump:                      # a jump up: a gap opens after the obstacle at i
                if start is None:
                    start = i
            elif step < -self.jump:                   # a jump down: the gap closes at the obstacle at i + 1
                if start is not None:
                    regions.append((start, i + 1))
                    start = None
        out = []
        for a, b in regions:                          # ext indices -> real sonar indices (ext = real + 1)
            ia, ib = a - 1, b - 1                     # -1 and n mean "the pretend wall"
            ra, rb = ext[a], ext[b]
            aa = ang(ia)
            ab = ang(ib)
            # width of the opening between the two obstacle edges
            width = math.sqrt(max(0.0, ra * ra + rb * rb - 2 * ra * rb * math.cos(ab - aa)))
            if ra <= 0.0 and rb <= 0.0:                # nothing but the pretend wall on both sides: the whole picture is open
                width = math.inf
            elif ra <= 0.0 or rb <= 0.0:               # one edge is the pretend wall: use the real obstacle's distance
                width = max(ra, rb) * abs(ab - aa)
            inner = [rs[k] for k in range(max(ia + 1, 0), min(ib, n))]
            if width >= self.min_width and inner and max(inner) >= depth_needed:
                out.append((ia, ib, ra, rb))
        return out

    # ------------------------------------------------------------------ the algorithm
    def step(self, p):
        front = p.sector_min(math.radians(-15), math.radians(15))
        if math.isnan(front):                          # never drive blind
            return Command(0.0, 0.0)

        n = len(p.ranges)
        rs = [self.look_max if math.isnan(r) else min(r, self.look_max) for r in p.ranges]

        def ang(i):
            return p.angle_min + i * p.angle_increment

        goal = p.goal_bearing()
        regions = self.find_regions(rs, ang, n, min(self.open_depth, max(0.30, p.goal_distance() + 0.10)))
        # is the way to the goal free? (the readings around the goal direction reach the goal, or at least `look_max`)
        around = [rs[i] for i in range(n) if abs(ang(i) - goal) <= math.radians(10)]
        goal_free = bool(around) and min(around) >= min(p.goal_distance(), self.look_max) - 0.10

        # ---- choose the free walking area: the gap with the edge direction nearest the goal ----
        best = None
        for ia, ib, ra, rb in regions:
            la, lb = ang(ia), ang(ib)                  # the gap lies between the directions la and lb (la < lb)
            if la < goal < lb and goal_free:
                cost, inside = 0.0, True
            else:
                cost, inside = min(abs(goal - la), abs(goal - lb)), False
            mid = 0.5 * (la + lb)
            if self.last_side and mid * self.last_side < -0.05:
                cost += self.side_bonus
            if best is None or cost < best[0]:
                best = (cost, ia, ib, ra, rb, inside)

        if best is None:                               # no gap at all: turn on the spot towards the roomier side
            if self.spin_dir == 0:
                left = sum(rs[i] for i in range(n) if ang(i) > 0)
                right = sum(rs[i] for i in range(n) if ang(i) < 0)
                self.spin_dir = 1 if left >= right else -1
            cmd = Command(0.0, self.spin_dir * self.spin_speed)
            self.print_debug(p, goal, 'no gap, spinning', cmd)
            return cmd

        _, ia, ib, ra, rb, inside = best
        la, lb = ang(ia), ang(ib)
        width_angle = lb - la

        # ---- the safety zone: is anything closer than `safety` on the left or on the right? ----
        dl = min([rs[i] for i in range(n) if ang(i) > 0.02] or [self.look_max])
        dr = min([rs[i] for i in range(n) if ang(i) < -0.02] or [self.look_max])
        dc = min([rs[i] for i in range(n) if abs(ang(i)) <= 0.02] or [self.look_max])
        if dc < self.safety:
            dl, dr = (min(dl, dc), dr) if dl <= dr else (dl, min(dr, dc))      # the centre slot counts for the closer side
        left_close, right_close = dl < self.safety, dr < self.safety

        # ---- what would we do if we were safe (high-safety rules)? ----
        def edge_heading(side):
            """aim along the gap's edge on `side` (+1 left edge, -1 right edge), a bit inside to miss the obstacle"""
            if side > 0:
                r_edge, a_edge = ra, la
            else:
                r_edge, a_edge = rb, lb
            shift = math.asin(min(1.0, self.clear / max(r_edge, self.clear))) + math.radians(5)
            shift = min(shift, 0.5 * width_angle)
            return a_edge + (shift if side > 0 else -shift)

        if inside:
            hs_heading, situation = goal, 'HSGR goal in the gap'
        elif width_angle > self.wide_angle:
            side = 1 if abs(goal - la) <= abs(goal - lb) else -1
            hs_heading, situation = edge_heading(side), 'HSWR wide gap, along its edge'
        else:
            hs_heading, situation = 0.5 * (la + lb), 'HSNR narrow gap, through the middle'

        # ---- the five situations ----
        if left_close and right_close:                 # LS2: squeeze through the middle, towards the roomier side
            heading = max(-0.7, min(0.7, 1.2 * (dl - dr) / max(dl + dr, 1e-6)))
            situation = 'LS2 both sides close, through the middle'
            slow = 0.5
        elif left_close or right_close:                # LS1: steer away from the close obstacle, more the closer it is
            s = -1 if left_close else 1                # +1 = turn left, -1 = turn right
            d = dl if left_close else dr
            k = max(0.0, min(1.0, (self.safety - d) / (self.safety - 0.10)))
            away = s * math.radians(60)
            heading = (1.0 - k) * hs_heading + k * away
            situation = 'LS1 close on the ' + ('left' if left_close else 'right') + ', steer away'
            slow = 0.6
        else:
            heading = hs_heading
            slow = 1.0
        heading = max(-math.pi / 2, min(math.pi / 2, heading))
        if abs(heading) > 0.05:
            self.last_side = 1 if heading > 0 else -1

        # ---- drive ----
        if self.spin_dir != 0 and abs(heading) > 0.6 * self.sharp:
            cmd = Command(0.0, self.spin_dir * self.spin_speed)
        elif abs(heading) > self.sharp:
            self.spin_dir = 1 if heading > 0 else -1
            cmd = Command(0.0, self.spin_dir * self.spin_speed)
        else:
            self.spin_dir = 0
            room = max(0.0, min(1.0, (front - 0.20) / 0.40))
            speed = max(self.min_speed, self.cruise_speed * room * math.cos(heading) * slow)
            cmd = Command(speed, self.turn_gain * heading)
        self.print_debug(p, goal, f'{situation}: steer {math.degrees(heading):+.0f} deg', cmd)
        return cmd

    def print_debug(self, p, goal, mode, cmd):
        if self.debug and p.time - self.last_print >= 1.0:
            self.last_print = p.time
            print(f'[nd] t={p.time:.1f}s goal={math.degrees(goal):+.0f} deg {mode} '
                  f'-> speed={cmd.linear:.2f} turn={cmd.angular:+.2f}', flush=True)
