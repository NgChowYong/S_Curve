# S_Curve for velocity control

## Python Demo

use maximum acceleration and maximum jerk to limit controller output for smoother velocity profile output
output:
![image](https://github.com/NgChowYong/S_Curve/blob/main/result.png)

# parameter to be set:
max speed
max acceleration
max jerk
scale (for smoother output, but will cause delay)
vel_threshold (stable output near desire vel)
acc_threshold (avoid overshoot of acceleration)


