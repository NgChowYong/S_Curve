from matplotlib import pyplot as plt
from simulation import SCurve

# vel command
vel_input = [1, 1.5, 1.5, 1.5, -1.0, 0.5, 0.3, 1.1, 1.15, 1.5, -2., 1.8, 2, 0.1, 0]  # all in m/s

# initialize
new_input = 0   # initial input
vel = 0         # initial vel
last_new_input = new_input

# for plotting
las_vel = 0
vel_output = []
vel_output2 = []

# create object and initialize
sc = SCurve()
sc.setup(new_input)


def compute_acc_and_jerk(vel_output_):
    last_v = vel_output_[0]
    last_a = 0
    acc_output_ = []
    jerk_output_ = []
    for i in vel_output_:
        a = (i - last_v) * 1000
        j = (a - last_a) * 1000
        acc_output_.append(a)
        jerk_output_.append(j)
        last_v = i
        last_a = a
    return acc_output_, jerk_output_


def plot_vel_acc_jerk(vel_output_, vel_output2_, acc_output_, jerk_output_):
    fig, axs = plt.subplots(3, 1, sharex='all')
    axs[0].set_title('velocity and desire velocity')
    axs[0].plot(vel_output_, label="vel")
    axs[0].plot(vel_output2_, label="desire_vel")
    axs[0].legend()
    axs[0].grid()
    axs[1].set_title('acceleration')
    axs[1].plot(acc_output_, label="acc")
    axs[1].legend()
    axs[1].grid()
    axs[2].set_title('jerk')
    axs[2].plot(jerk_output_, label="jerk")
    axs[2].legend()
    axs[2].grid()
    plt.show()


############ START SIMULATION ############
# simulation time goes from 0 to 1500 ms = 0 s to 1.5 s for simulation
simulation_time = 1100  # in ms
for i in range(simulation_time):

    # frequency to get a new velocity input
    # example: get input in 10 hz = 0.1s = 100ms, then i = 100
    if i % 70 == 0:
        # check for available input
        if len(vel_input) > 0:
            last_new_input = new_input
            new_input = vel_input.pop(0)
        else:
            new_input = 0

        # reject repeated input
        if new_input != last_new_input:
            sc.setup(new_input)

    # frequency of control in ms
    delta_t = 1  # 1 ms
    if i % delta_t == 0:
        las_vel = vel
        vel = sc.control(vel, delta_t / 1000)  # update velocity

    # for plotting
    vel_output.append(vel)
    vel_output2.append(new_input)
    # plt.plot([i, i+1], [las_vel, vel])
    # plt.pause(0.0001)

# compute acceleration and jerk
# plot out velocity, acceleration and jerk output
[acc_output, jerk_output] = compute_acc_and_jerk(vel_output)
plot_vel_acc_jerk(vel_output, vel_output2, acc_output, jerk_output)
