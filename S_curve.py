from matplotlib import pyplot as plt

# vel command
vel_input = [1,1.5,1.0,0.5,0.3,1.1,1.15,1.5,1.05,1.8,2,0] # all in m/s


class scurve:
    def __init__(self):
        self.max_speed = 20 # in m / s^2
        self.max_acc = 80 # in m / s^2
        self.max_jerk = 5000 # in m / s^2
        self.desire_speed = 0
        self.start_speed = 0
        self.scale = 1.3

        self.vel_threshold = 0.01
        self.acc_threshold = 0.1
        self.vel = 0
        self.last_vel = 0
        self.acc_for_jerk = 0
        self.acc = 0
        self.last_acc = 0

        self.state = 0
        self.threshold_for_slow = 0
        self.half = 0

    def setup(self, start_speed, desire_speed):
        self.desire_speed = desire_speed # in m / s
        self.start_speed = start_speed
        self.half = (desire_speed + start_speed) / 2

    def control(self, curr_vel, delta_time):
        self.acc = (curr_vel - self.last_vel) / delta_time

        # if velocity reach just output velocity
        if abs(self.desire_speed - curr_vel) < self.vel_threshold:
            if self.state != 0:
                self.state = 0
            self.vel = curr_vel
            self.acc_for_jerk = 0

        elif self.desire_speed > curr_vel:
            if curr_vel < self.half:
                # do acc
                # acc depends on acc value
                #if self.acc < self.max_acc * 0.8:
                if self.state != 2:
                    self.acc_for_jerk = 0
                    # self.acc_for_jerk = self.acc_for_jerk/2
                    self.state = 2
                    # constant jerk increasse acc
                self.acc_for_jerk += self.max_jerk * delta_time
                if self.acc_for_jerk > self.max_acc:
                    self.acc_for_jerk = self.max_acc
                self.vel = curr_vel + self.acc_for_jerk * delta_time
            else:
                # dcc
                if self.state == 1 or self.state == 2:
                    #self.acc_for_jerk = self.acc_for_jerk
                    self.threshold_for_slow = self.acc_for_jerk * self.acc_for_jerk / (2 * self.max_jerk) * self.scale
                    #self.state = 1
                if self.desire_speed - curr_vel <= self.threshold_for_slow:
                    if self.state != 4:
                        self.state = 4
                    # decrease speed with constant jerk
                    self.acc_for_jerk -= self.max_jerk * delta_time
                    # small increment
                    if self.acc_for_jerk <= self.acc_threshold:
                        self.acc_for_jerk = self.acc_threshold
                    self.vel = curr_vel + self.acc_for_jerk * delta_time
                else:
                    if self.state != 3:
                        self.state = 3
                    # linear acceleration
                    self.acc_for_jerk += self.max_jerk * delta_time
                    if self.acc_for_jerk > self.max_acc:
                        self.acc_for_jerk = self.max_acc
                    self.vel = curr_vel + self.acc_for_jerk * delta_time
        else:
            # decreasing speed
            # first half
            if curr_vel > self.half:
                # do dcc, depends on dcc value
                if self.state != 2:
                    self.acc_for_jerk = 0
                    self.state = 2
                # constant jerk increasse dcc
                self.acc_for_jerk -= self.max_jerk * delta_time
                if self.acc_for_jerk < -self.max_acc:
                    self.acc_for_jerk = -self.max_acc
                self.vel = curr_vel + self.acc_for_jerk * delta_time
                # reach max acc
            else:
                # dcc
                if self.state == 1 or self.state == 2:
                    self.threshold_for_slow = (self.acc_for_jerk * self.acc_for_jerk / (2 * self.max_jerk)) * self.scale
                if curr_vel - self.desire_speed <= self.threshold_for_slow:
                    if self.state != 4:
                        self.state = 4
                    # decrease speed with constant jerk
                    self.acc_for_jerk += self.max_jerk * delta_time
                    if self.acc_for_jerk >= -self.acc_threshold:
                        self.acc_for_jerk = -self.acc_threshold
                    self.vel = curr_vel + self.acc_for_jerk * delta_time
                else:
                    if self.state != 3:
                        self.state = 3
                    # linear acceleration
                    self.acc_for_jerk -= self.max_jerk * delta_time
                    if self.acc_for_jerk < -self.max_acc:
                        self.acc_for_jerk = -self.max_acc
                    self.vel = curr_vel + self.acc_for_jerk * delta_time
                    
        if self.vel >= self.max_speed:
            self.vel = self.max_speed
        elif self.vel <= -self.max_speed:
            self.vel = -self.max_speed
            
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

vel_output = []
vel_output2 = []

# time goes from 0 to 1500 ms = 1.5 s for simulation
for i in range(500):
    # example: get input in 10 hz = 0.1s = 100ms, then i = 100
    if i % 30 == 0: # get new message
        if len(vel_input) > 0:
            last_new_input = new_input
            new_input = vel_input.pop(0)
        if new_input != last_new_input:
            sc.setup(vel, new_input)

    # 1 ms control rate
    delta_t = 1 # 1 ms
    if i % delta_t == 0: # get new message
        las_vel = vel
        vel = sc.control(vel, delta_t/1000)

    vel_output.append(vel)
    vel_output2.append(new_input)
    # plt.plot([i, i+1], [las_vel, vel])
    # plt.pause(0.001)

last_v = vel_output[0]
last_a = 0
acc_output = []
jerk_output = []
for i in vel_output:
    a = (i-last_v)*1000
    j = (a-last_a)*1000
    acc_output.append(a)
    jerk_output.append(j)
    last_v = i
    last_a = a

##plt.show()
##
##plt.figure(1)
##plt.plot(vel_output,label="vel")
##plt.plot(vel_output2,label="desire_vel")
##plt.figure(2)
##plt.plot(acc_output,label="acc")
##plt.figure(3)
##plt.plot(jerk_output,label="jerk")
##plt.legend()
##plt.show()

fig, axs = plt.subplots(3, 1, sharex='all')
axs[0].plot(vel_output,label="vel")
axs[0].plot(vel_output2,label="desire_vel")
axs[1].plot(acc_output,label="acc")
axs[2].plot(jerk_output,label="jerk")
plt.show()
