"""Algorithm 4: fuzzy-logic avoidance.

Idea: instead of hard yes/no tests ("closer than 40 cm?"), the robot uses soft words
like NEAR, MEDIUM and FAR. Something 30 cm away is "mostly near, a bit medium".
A small list of plain-language rules says what to do for each word, and the robot
blends the rules that apply, weighted by how true each one is.

This is a Takagi-Sugeno fuzzy controller: every rule gives a fixed turn rate and a
fixed speed, and the final command is their weighted average.
"""
import math
import os

from obs_avoid_control.core.interface import Algorithm, Command


def clip01(x):
    return max(0.0, min(1.0, x))


class Controller(Algorithm):
    name = 'fuzzy'

    def __init__(self):
        # --- what the robot looks at (angles in the sonar picture) ---
        self.front_half = math.radians(20)     # "front" is +-20 degrees
        self.side_inner = math.radians(22.5)   # "left" is 25..75 degrees (no overlap with front)
        self.side_outer = math.radians(75)     # "right" is -75..-25 degrees
        self.open_default = 2.0                # a side with no readings counts as open

        # --- soft words for distance (metres) ---
        # front: NEAR = 1 up to 0.25, fades out at 0.50.   FAR = 0 up to 0.50, 1 from 0.80.
        self.front_near_full, self.front_near_zero = 0.25, 0.50
        self.front_far_start, self.front_far_full = 0.50, 0.80
        # sides: NEAR = 1 up to 0.25, fades out at 0.55
        self.side_near_full, self.side_near_zero = 0.25, 0.55

        # --- outputs of the rules ---
        self.fast, self.medium, self.slow = 0.20, 0.12, 0.05     # m/s
        self.gentle, self.firm, self.hard = 0.4, 0.6, 0.9         # rad/s

        self.debug = os.environ.get('ALGO_DEBUG') == '1'
        self.last_print = -10.0

    def reset(self):
        self.last_print = -10.0

    def step(self, p):
        front = p.sector_min(-self.front_half, self.front_half)
        if math.isnan(front):                    # never drive blind
            return Command(0.0, 0.0)
        left = p.sector_min(self.side_inner, self.side_outer)
        right = p.sector_min(-self.side_outer, -self.side_inner)
        if math.isnan(left):
            left = self.open_default
        if math.isnan(right):
            right = self.open_default

        # ---- step 1: turn numbers into soft words (each between 0 and 1) ----
        near_f = clip01((self.front_near_zero - front) / (self.front_near_zero - self.front_near_full))
        far_f = clip01((front - self.front_far_start) / (self.front_far_full - self.front_far_start))
        med_f = 1.0 - near_f - far_f

        span = self.side_near_zero - self.side_near_full
        near_l = clip01((self.side_near_zero - left) / span)
        near_r = clip01((self.side_near_zero - right) / span)

        # which side is roomier?  (positive = left has more room)
        room = near_r - near_l
        left_more = clip01(room / 0.5)
        right_more = clip01(-room / 0.5)
        same = 1.0 - max(left_more, right_more)          # on a tie the robot prefers LEFT

        bearing = p.goal_bearing()                       # positive = goal is on the left
        goal_left = clip01((bearing - 0.1) / 0.5)
        goal_right = clip01((-bearing - 0.1) / 0.5)
        goal_ahead = 1.0 - max(goal_left, goal_right)

        # ---- step 2: the rules: (how true it is, turn rate, speed) ----
        rules = [
            (min(far_f, goal_left),    +self.gentle, self.fast),    # 1 clear + goal left  -> turn left
            (min(far_f, goal_ahead),    0.0,         self.fast),    # 2 clear + goal ahead -> straight
            (min(far_f, goal_right),   -self.gentle, self.fast),    # 3 clear + goal right -> turn right
            (near_l,                   -self.firm,   self.medium),  # 4 left near  -> veer right
            (near_r,                   +self.firm,   self.medium),  # 5 right near -> veer left
            (min(med_f, left_more),    +self.gentle, self.medium),  # 6 front medium, left roomier
            (min(med_f, right_more),   -self.gentle, self.medium),  # 7 front medium, right roomier
            (min(med_f, same),         +self.gentle, self.medium),  # 8 front medium, tie -> left
            (min(near_f, left_more),   +self.hard,   self.slow),    # 9 front near, left roomier
            (min(near_f, right_more),  -self.hard,   self.slow),    # 10 front near, right roomier
            (min(near_f, same),        +self.hard,   self.slow),    # 11 front near, tie -> left
        ]

        # ---- step 3: blend the rules (weighted average) ----
        total = sum(w for w, _, _ in rules)
        if total < 1e-6:
            return Command(self.slow, 0.0)
        turn = sum(w * t for w, t, _ in rules) / total
        speed = sum(w * s for w, _, s in rules) / total

        if self.debug and p.time - self.last_print >= 1.0:
            self.last_print = p.time
            print(f'[fuzzy] t={p.time:.1f}s front={front:.2f} left={left:.2f} right={right:.2f} '
                  f'-> speed={speed:.2f} turn={turn:+.2f}', flush=True)

        return Command(speed, turn)
