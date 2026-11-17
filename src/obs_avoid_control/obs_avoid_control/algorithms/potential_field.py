"""Algorithm 5: Potential field (attractive goal + repulsive obstacles), with a way out of the "stuck" spot.

Idea: the goal PULLS the robot like a magnet. Every obstacle the sonar sees PUSHES it away, harder the closer it is.
Add all the pushes and the pull together: the arrow that comes out is the way to drive.

The classic problem: when an obstacle sits right between the robot and the goal, the push (backwards) and the pull
(forwards) are equal and opposite. They cancel, the arrow is zero, and the robot sits still forever. This is a "local
minimum". This version has three cures:

  1. SWIRL     Part of every push is turned sideways (always to the same side, the one with more room). A push that
               points straight back can never cancel the pull exactly; it now slides the robot around the obstacle.
  2. STALL     If the distance to the goal has not got smaller for `stall_time` seconds, the robot is stuck.
               It then ignores the goal for a few seconds and walks out SIDEWAYS (a fixed compass direction),
               still being pushed away by the obstacles.
  3. SWITCH    Each stall makes it try the OTHER side next time, so it does not repeat the same mistake.
"""
import math
import os

from obs_avoid_control.core.interface import Algorithm, Command, wrap_angle


class Controller(Algorithm):
    name = 'potential_field'

    def __init__(self):
        self.cruise_speed = 0.20       # m/s
        self.min_speed = 0.04          # m/s
        self.k_att = 1.0               # strength of the goal's pull (constant: it does not fade near the goal)
        self.k_rep = float(os.environ.get('PF_REP', '0.35'))             # strength of the obstacles' push
        self.d0 = float(os.environ.get('PF_RANGE', '0.90'))              # m: obstacles farther than this do not push
        self.swirl = math.radians(float(os.environ.get('PF_SWIRL', '70')))   # how far each push is turned sideways
        self.stall_time = float(os.environ.get('PF_STALL', '5.0'))       # s without progress -> stuck
        self.stall_gain = 0.15         # m: "progress" means the goal got this much closer
        self.crawl_time = 2.0          # s of crawling right in front of something -> stuck
        self.escape_time = 4.0         # s of walking sideways after a stall
        self.turn_gain = 1.6           # rad/s per radian of heading error
        self.spin_speed = 0.7          # rad/s when turning on the spot
        self.sharp = math.radians(60)  # heading errors above this: turn on the spot
        self.debug = os.environ.get('ALGO_DEBUG') == '1'
        self.reset()

    def reset(self):
        self.last_print = -10.0
        self.side = 0                  # +1 swirl to the left, -1 to the right, 0 not chosen yet
        self.best_goal = None          # closest we have been to the goal in this window
        self.best_time = 0.0           # when we last got closer
        self.escape_until = -1.0
        self.escape_heading = 0.0      # compass direction (world frame) to walk while escaping
        self.stalls = 0
        self.spin_dir = 0

    # ------------------------------------------------------------------ the forces
    def forces(self, p, goal, swirl_side, use_goal):
        """Return (fx, fy) in the robot's frame: x forward, y left."""
        fx = fy = 0.0
        if use_goal:
            fx += self.k_att * math.cos(goal)
            fy += self.k_att * math.sin(goal)
        for i, r in enumerate(p.ranges):
            if math.isnan(r) or r >= self.d0:          # unknown or far: no push
                continue
            a = p.angle_min + i * p.angle_increment    # direction of the obstacle
            r = max(r, 0.10)
            push = self.k_rep * (1.0 / r - 1.0 / self.d0) / r
            away = a + math.pi                          # straight away from it
            # turn the push sideways so it slides the robot round the obstacle instead of straight back:
            # an obstacle clearly on one side pushes the robot to the other side,
            # an obstacle dead ahead pushes it to the chosen side (this is what breaks the tie)
            s = swirl_side if abs(a) < math.radians(10) else (-1 if a > 0 else 1)
            away -= s * self.swirl
            fx += push * math.cos(away) / 6.0           # /6: about six readings see the same obstacle
            fy += push * math.sin(away) / 6.0
        return fx, fy

    def pick_side(self, p):
        """Swirl towards the side with more room (sum of the distances on each side)."""
        left = right = 0.0
        for i, r in enumerate(p.ranges):
            a = p.angle_min + i * p.angle_increment
            v = 2.0 if math.isnan(r) else min(r, 2.0)
            if a > 0.02:
                left += v
            elif a < -0.02:
                right += v
        return 1 if left >= right else -1

    # ------------------------------------------------------------------ the algorithm
    def step(self, p):
        front = p.sector_min(math.radians(-15), math.radians(15))
        if math.isnan(front):                          # never drive blind
            return Command(0.0, 0.0)

        goal = p.goal_bearing()
        dist = p.goal_distance()

        # ---- stall check: is the goal getting closer? ----
        if self.best_goal is None or dist < self.best_goal - self.stall_gain:
            self.best_goal, self.best_time = dist, p.time
        escaping = p.time < self.escape_until
        # also stuck: the arrow points ahead but something is right in front, so we are crawling or stopped
        if front < 0.30 and not self.spinning:
            if self.crawl_since is None:
                self.crawl_since = p.time
        else:
            self.crawl_since = None
        crawling = self.crawl_since is not None and p.time - self.crawl_since > self.crawl_time
        if not escaping and dist > 0.3 and (p.time - self.best_time > self.stall_time or crawling):
            self.crawl_since = None
            self.stalls += 1
            self.side = -self.side if self.side else self.pick_side(p)     # try the OTHER side than last time
            self.escape_until = p.time + self.escape_time
            self.escape_heading = p.yaw + self.side * math.radians(90)      # walk sideways, in a fixed compass direction
            self.best_goal, self.best_time = dist, p.time + self.escape_time
            escaping = True
            if self.debug:
                print(f'[pf] t={p.time:.1f}s STUCK (stall #{self.stalls}): walking out to the '
                      f'{"left" if self.side > 0 else "right"}', flush=True)
        if self.side == 0:
            self.side = self.pick_side(p)

        # ---- add the pull and the pushes ----
        if escaping:
            # ignore the goal: pull along the escape direction instead
            fx, fy = self.forces(p, goal, self.side, use_goal=False)
            e = wrap_angle(self.escape_heading - p.yaw)
            fx += self.k_att * math.cos(e)
            fy += self.k_att * math.sin(e)
            mode = 'escaping'
        else:
            fx, fy = self.forces(p, goal, self.side, use_goal=True)
            mode = 'normal'
        heading = math.atan2(fy, fx)                    # the way the arrow points
        heading = max(-math.pi, min(math.pi, heading))

        # ---- drive ----
        if self.spin_dir != 0 and abs(heading) > 0.6 * self.sharp:      # keep turning the same way until nearly lined up
            cmd = Command(0.0, self.spin_dir * self.spin_speed)
            self.spinning = True
        elif abs(heading) > self.sharp:
            self.spin_dir = 1 if heading > 0 else -1
            cmd = Command(0.0, self.spin_dir * self.spin_speed)
            self.spinning = True
        else:
            self.spin_dir = 0
            self.spinning = False
            room = max(0.0, min(1.0, (front - 0.20) / 0.40))
            speed = max(self.min_speed, self.cruise_speed * room * math.cos(heading)) if front > 0.22 else 0.0
            cmd = Command(speed, self.turn_gain * heading)
        self.print_debug(p, goal, heading, mode, cmd)
        return cmd

    def print_debug(self, p, goal, heading, mode, cmd):
        if self.debug and p.time - self.last_print >= 1.0:
            self.last_print = p.time
            print(f'[pf] t={p.time:.1f}s goal={math.degrees(goal):+.0f} deg arrow={math.degrees(heading):+.0f} deg '
                  f'{mode} -> speed={cmd.linear:.2f} turn={cmd.angular:+.2f}', flush=True)
