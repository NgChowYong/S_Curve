# S_Curve for velocity smoothing

Algorithm for smoothing velocity command under constant acceleration and adaptive jerk consideration, used for motor control signal smoothing

Author: Ng Chow Yong

Latest version update date: 2021 - 02 - 19

## Parameters
Parameters needed to be tuned:

- max velocity: physical maximum velocity that motor can reach

- max acceleration: desired maximum acceleration (to be fine tuned) or can be set as physical maximum acceleration that motor can reach

- max jerk: desired maximum jerk (to be fine tuned)

- vel_threshold: velocity to avoid jiggle velocity output near desire output, small value range is enough, too small will not be reach due to imprecise of quatization

- acc_threshold: to avoid negative sign acceleration output, a near zero positive value is enough

- acc_jiggle_threshold: to reduce jiggle acceleration curve due to quantization error, more than 1 is desired for smoother curve

# Theory

The idea is to use maximum acceleration and maximum jerk to limit controller output for smoother velocity profile output given a sequence of velocity command input.

In order to generate smooth velocity curve, the curve can be separated into acceleration region and deceleration region. Acceleration region will increase velocity under constant jerk, using accumulated acceleration to increase current velocity. 

As for deceleration region, when to turn into deceleration region is crucial and needed to be determine. According to equation below:

v<sub>&delta;</sub> = a^2 / 2*j<sub>max;</sub>

After v<sub>&delta;</sub> is obtained, to avoid digital integral error, update of jerk is necessarily for since constant jerk will reach the desire velocity only in ideal condition. The update of jerk is based on same equation but determined by current difference of velocity. A threshold for acceleration_jiggle is introduced to avoid jiggle due to numerical imprecision.

The overall output is shown below:

# Output:
Output of the simulation will show velocity, acceleration and jerk of simulation, a smooth velocity curve and less jiggle acceleration curve is desired.

![image](https://github.com/NgChowYong/S_Curve/blob/main/result.png)

