from matplotlib import pyplot as plt

# vel command
vel_input = [1,1.5,1.5,1.5,1.0,0.5,0.3,1.1,1.15,1.5,1.05,1.8,2,0.1,0] # all in m/s
# vel_input = [1,4,0.2,0.1,0] # all in m/s


class scurve:
    def __init__(self):
        self.max_speed = 20# in m / s^2
        self.max_acc = 100# in m / s^2
        self.max_jerk = 5000 # in m / s^2
        self.desire_speed = 0
        self.scale = 1

        self.vel_threshold = 0.01
        self.acc_threshold = 0.001
        self.vel = 0
        self.last_vel = 0
        self.acc_for_jerk = 0
        self.dec_for_jerk = 0
        self.acc = 0
        self.last_acc = 0

        self.state = 0
        self.threshold_for_slow = 0
        self.jerk = 0

    def setup(self, desire_speed):
        self.desire_speed = desire_speed # in m / s
        self.state = 0

    def control(self, curr_vel, delta_time):
        self.acc = (curr_vel - self.last_vel) / delta_time
        self.jerk = self.acc - self.last_acc

        # if velocity reach just output velocity
        if abs(self.desire_speed - curr_vel) < self.vel_threshold:
            if self.state != 0:
                self.state = 0
            self.vel = self.desire_speed
            self.acc_for_jerk = 0
            self.dec_for_jerk = 0

        elif self.desire_speed > curr_vel:
            if self.state == 2:
                self.vel = self.desire_speed
            self.state = 1
            self.dec_for_jerk = 0
            self.threshold_for_slow = self.acc * self.acc / (2 * self.max_jerk) * self.scale
            if self.desire_speed - curr_vel <= self.threshold_for_slow:
                self.acc_for_jerk -= self.acc * self.acc/(2*(self.desire_speed - curr_vel)) * delta_time
                # self.acc_for_jerk -= self.max_jerk * delta_time
                # small increment
                if self.acc_for_jerk <= self.acc_threshold:
                    self.acc_for_jerk = self.acc_threshold
                self.vel = curr_vel + self.acc_for_jerk * delta_time
            else:
                # linear acceleration
                self.acc_for_jerk += self.max_jerk * delta_time
                if self.acc_for_jerk > self.max_acc:
                    self.acc_for_jerk = self.max_acc
                self.vel = curr_vel + self.acc_for_jerk * delta_time
        else:
            if self.state == 1:
                self.vel = self.desire_speed
            self.state = 2
            # decreasing speed
            self.acc_for_jerk = 0
            self.threshold_for_slow = (self.acc * self.acc / (2 * self.max_jerk)) * self.scale
            if curr_vel - self.desire_speed <= self.threshold_for_slow:
                # decrease speed with constant jerk
                self.dec_for_jerk += self.acc * self.acc/(2*(curr_vel - self.desire_speed)) * delta_time
                # self.dec_for_jerk += self.max_jerk * delta_time
                if self.dec_for_jerk >= -self.acc_threshold:
                    self.dec_for_jerk = -self.acc_threshold
                self.vel = curr_vel + self.dec_for_jerk * delta_time
            else:
                # linear acceleration
                self.dec_for_jerk -= self.max_jerk * delta_time
                if self.dec_for_jerk < -self.max_acc:
                    self.dec_for_jerk = -self.max_acc
                self.vel = curr_vel + self.dec_for_jerk * delta_time
                    
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
sc.setup(new_input)
# plot
las_vel = 0

vel_output = []
vel_output2 = []

# time goes from 0 to 1500 ms = 1.5 s for simulation
for i in range(1000):
    # example: get input in 10 hz = 0.1s = 100ms, then i = 100
    if i % 70 == 0: # get new message
        if len(vel_input) > 0:
            last_new_input = new_input
            new_input = vel_input.pop(0)
        else:
            new_input = 0
            
        if new_input != last_new_input:
            sc.setup(new_input)
            
    # 1 ms control rate
    delta_t = 1 # 1 ms
    if i % delta_t == 0: # get new message
        las_vel = vel
        vel = sc.control(vel, delta_t/1000)

    vel_output.append(vel)
    vel_output2.append(new_input)
    # plt.plot([i, i+1], [las_vel, vel])
    # plt.pause(0.0001)

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

fig, axs = plt.subplots(3, 1, sharex='all')
axs[0].set_title('velocity and desire velocity')
axs[0].plot(vel_output,label="vel")
axs[0].plot(vel_output2,label="desire_vel")
axs[0].legend()
axs[0].grid()
axs[1].set_title('acceleration')
axs[1].plot(acc_output,label="acc")
axs[1].legend()
axs[1].grid()
axs[2].set_title('jerk')
axs[2].plot(jerk_output,label="jerk")
axs[2].legend()
axs[2].grid()
plt.show()
