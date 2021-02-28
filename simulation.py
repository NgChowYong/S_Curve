# Algorithm for smoothing velocity command under constant acceleration and adaptive jerk consideration
# Author: Ng Chow Yong
# Latest-Update-Date: 2021 - 02 - 28
# Description:
#   The theory is based on constant jerk to compute a smooth velocity curve, details is written in README.md
#   2 functions are used, 1 is to update desire velocity, 1 is to compute output velocity based on current velocity
#   parameters need to be set:
#       max velocity: physical maximum velocity
#       max acceleration: desire maximum acceleration (to be fine tuned)
#       max jerk: desire maximum jerk (to be fine tuned)
#       vel_threshold: velocity to avoid jiggle velocity output near desire output
#       acc_threshold: to avoid negative sign acceleration output 
#       acc_jiggle_threshold: to reduce jiggle acceleration curve due to quantization error


class SCurve:
    def __init__(self):
        self.max_velocity = 20  # in m / s^2
        self.max_acceleration = 100  # in m / s^2
        self.max_jerk = 5000  # in m / s^2
        self.vel_threshold = 0.01
        self.acc_threshold = 0.001
        self.acc_jiggle_threshold = 1.2
        
        self.last_vel = 0
        self.last_acc = 0
        self.acc = 0
        self.dec = 0
        self.state = 0

        self.desire_speed = 0

    def setup(self, desire_speed):
        # setup speed and reset state
        self.desire_speed = desire_speed  # in m / s
        self.state = 0

    def control(self, curr_vel, delta_time):
        # compute current acceleration
        curr_acc = (curr_vel - self.last_vel) / delta_time
        vel_output = 0
        
        # if velocity reached just output desire velocity (no need of control)
        if abs(self.desire_speed - curr_vel) < self.vel_threshold:
            if self.state != 0:
                self.state = 0
            vel_output = self.desire_speed
            self.acc = 0
            self.dec = 0

        elif self.desire_speed > curr_vel:
            # if change of state means overshoot, limit output for overshoot
            if self.state == 2:
                vel_output = self.desire_speed
                
            # update state
            self.state = 1
            self.dec = 0

            # compute range of velocity for deceleration
            slow_down_threshold = curr_acc * curr_acc / (2 * self.max_jerk)
            if self.desire_speed - curr_vel <= slow_down_threshold:
                # from current acceleration compute compensation jerk value
                self.acc -= curr_acc * curr_acc / \
                            (2 * (self.desire_speed - curr_vel) * self.acc_jiggle_threshold) * delta_time
                # small increment
                if self.acc <= self.acc_threshold:
                    self.acc = self.acc_threshold
                vel_output = curr_vel + self.acc * delta_time
            else:
                # linear acceleration
                self.acc += self.max_jerk * delta_time

                # constant acceleration
                if self.acc > self.max_acceleration:
                    self.acc = self.max_acceleration
                vel_output = curr_vel + self.acc * delta_time
        else:
            # deceleration is same as acceleration but with different sign only

            if self.state == 1:
                vel_output = self.desire_speed
            self.state = 2

            # decreasing speed
            self.acc = 0
            slow_down_threshold = (curr_acc * curr_acc / (2 * self.max_jerk))
            if curr_vel - self.desire_speed <= slow_down_threshold:
                # decrease speed with adaptive jerk
                self.dec += curr_acc * curr_acc / \
                            (2 * (curr_vel - self.desire_speed) * self.acc_jiggle_threshold) * delta_time
                if self.dec >= -self.acc_threshold:
                    self.dec = -self.acc_threshold
                vel_output = curr_vel + self.dec * delta_time
            else:
                # linear acceleration
                self.dec -= self.max_jerk * delta_time
                # constant deceleration
                if self.dec < -self.max_acceleration:
                    self.dec = -self.max_acceleration
                vel_output = curr_vel + self.dec * delta_time

        # limit output
        if vel_output >= self.max_velocity:
            vel_output = self.max_velocity
        elif vel_output <= -self.max_velocity:
            vel_output = -self.max_velocity

        # record last output
        self.last_vel = curr_vel
        self.last_acc = curr_acc

        return vel_output
