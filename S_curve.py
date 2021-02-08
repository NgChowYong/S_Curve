from matplotlib import pyplot as plt

# vel command
vel_input = [1,1.5,1.0,0.5,0.3,1.1,1.15,1.5,1.05,1.8,2,0] # all in m/s


class scurve:
    def __init__(self):
        self.max_speed = 1 # in m / s^2
        self.max_acc = 0.1 # in m / s^2
        self.max_jerk = 0.02
        self.delta_time = 1
        self.desire_speed = 0
        self.start_speed = 0

        self.vel_threshold = 0.001
        self.acc_threshold = 0.01
        self.vel = 0
        self.last_vel = 0
        self.acc_for_jerk = 0
        self.acc = 0
        self.last_acc = 0

        self.state = 0
        self.threshold_for_slow = self.max_acc * self.max_acc / (2 * self.max_jerk)

        self.half = 0

    def setup(self, start_speed, desire_speed):
        self.desire_speed = desire_speed # in m / s
        self.start_speed = start_speed
        self.half = (desire_speed + start_speed) / 2

    def control(self, curr_vel):
        self.acc = (curr_vel - self.last_vel) / self.delta_time

        # if velocity reach just output velocity
        if abs(self.desire_speed - curr_vel) < self.vel_threshold:
            if self.state != 0:
                self.state = 0
            self.vel = curr_vel

        elif self.desire_speed > curr_vel:
            if curr_vel < self.half:
                # do acc
                # acc depends on acc value
                if self.acc < self.max_acc * 0.8:
                    if self.state != 2:
                        # self.acc_for_jerk = self.acc_for_jerk/2
                        self.state = 2
                    # constant jerk increasse acc
                    self.acc_for_jerk += self.max_jerk * self.delta_time
                    self.vel = curr_vel + self.acc_for_jerk * self.delta_time
                # reach max acc
                else:
                    if self.state != 3:
                        self.state = 3
                    # linear acceleration
                    self.acc_for_jerk = self.max_acc
                    self.vel = curr_vel + self.acc_for_jerk * self.delta_time
            else:
                # dcc
                if self.desire_speed - curr_vel <= self.threshold_for_slow:
                    if self.state != 1:
                        # self.acc_for_jerk = self.acc_for_jerk/2
                        self.state = 1
                    # decrease speed with constant jerk
                    self.acc_for_jerk -= self.max_jerk * self.delta_time
                    if self.acc_for_jerk <= self.acc_threshold:
                        self.acc_for_jerk = self.acc_threshold
                    self.vel = curr_vel + self.acc_for_jerk * self.delta_time
                    if self.vel >= self.desire_speed:
                        self.vel = self.desire_speed
                else:
                    if self.state != 3:
                        self.state = 3
                    # linear acceleration
                    self.acc_for_jerk = self.max_acc
                    self.vel = curr_vel + self.acc_for_jerk * self.delta_time
        else:
            # decreasing speed
            # first half
            if curr_vel > self.half:
                # do dcc, depends on dcc value
                if self.acc > -self.max_acc * 0.8:
                    if self.state != 2:
                        self.acc_for_jerk = 0
                        self.state = 2
                    # constant jerk increasse dcc
                    self.acc_for_jerk -= self.max_jerk * self.delta_time
                    self.vel = curr_vel + self.acc_for_jerk * self.delta_time
                # reach max acc
                else:
                    if self.state != 3:
                        self.state = 3
                    # linear acceleration
                    self.acc_for_jerk = -self.max_acc
                    self.vel = curr_vel + self.acc_for_jerk * self.delta_time
            else:
                # dcc
                if curr_vel - self.desire_speed <= self.threshold_for_slow:
                    if self.state != 1:
                        self.acc_for_jerk = self.acc
                        self.state = 1
                    # decrease speed with constant jerk
                    self.acc_for_jerk += self.max_jerk * self.delta_time
                    if self.acc_for_jerk >= self.acc_threshold:
                        self.acc_for_jerk = self.acc_threshold
                    self.vel = curr_vel + self.acc_for_jerk * self.delta_time
                    if self.vel <= self.desire_speed:
                        self.vel = self.desire_speed
                else:
                    if self.state != 3:
                        self.state = 3
                    # linear acceleration
                    self.acc_for_jerk = -self.max_acc
                    self.vel = curr_vel + self.acc_for_jerk * self.delta_time
        self.last_vel = curr_vel
        self.last_acc = self.acc
        return self.vel

# initialize
sc = scurve()
new_input = 0 # initial input
last_new_input = new_input
vel = 0 # current vel # all in m/s
sc.setup(vel, new_input)
# plot
las_vel = 0

# time goes from 0 to 1500 ms = 1.5 s for simulation
for i in range(1200):
    # get input in 10 hz = 0.1s = 100ms
    if i % 10 == 0: # get new message
        if len(vel_input) > 0:
            last_new_input = new_input
            new_input = vel_input.pop(0)
        if new_input != last_new_input:
            sc.setup(vel, new_input)

    # 1 ms control rate
    las_vel = vel
    vel = sc.control(vel)

    plt.grid()
    plt.plot([i, i+1], [las_vel, vel])
    plt.pause(0.001)

plt.show()