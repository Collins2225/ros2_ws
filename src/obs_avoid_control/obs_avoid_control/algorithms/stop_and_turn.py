import math

from obs_avoid_control.core.interface import Algorithm, Command


class Controller(Algorithm):
    """Algorithm 1: drive toward the goal. If something is close ahead, stop and
    turn on the spot toward the side with more room, until the way ahead is clear."""
    name = 'stop_and_turn'

    def __init__(self):
        self.stop_distance = 0.40     # too close: stop and turn
        self.clear_distance = 0.60    # need this much room ahead before driving again
        self.cruise_speed = 0.20
        self.turn_speed = 0.8
        self.steer_gain = 1.0         # how strongly to steer toward the goal
        self.turning = 0              # 0 = driving, +1 = turning left, -1 = turning right

    def reset(self):
        self.turning = 0

    def step(self, p):
        front = p.sector_min(math.radians(-15), math.radians(15))
        if math.isnan(front):
            return Command(0.0, 0.0)              # have not looked ahead yet: wait

        if self.turning == 0 and front < self.stop_distance:
            left = p.sector_mean(math.radians(15), math.radians(75))
            right = p.sector_mean(math.radians(-75), math.radians(-15))
            left = 0.0 if math.isnan(left) else left      # unknown counts as "no room"
            right = 0.0 if math.isnan(right) else right
            self.turning = 1 if left >= right else -1

        if self.turning != 0:
            if front >= self.clear_distance:
                self.turning = 0                  # the way is clear again
            else:
                return Command(0.0, self.turning * self.turn_speed)

        turn = self.steer_gain * p.goal_bearing()     # drive, steering toward the goal
        turn = max(-self.turn_speed, min(self.turn_speed, turn))
        return Command(self.cruise_speed, turn)
