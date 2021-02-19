# S_Curve for velocity control

###### Author: Ng Chow Yong
###### Date: 2021 - 02 - 19

## Details
the idea is to use maximum acceleration and maximum jerk to limit controller output for smoother velocity profile output given a sequence of velocity command input.

given arbitrary velocity input, determine its region (constant jerk region or contant acceleration or constant velocity region), then output velocity according to the region.

acceleration part is quite instinct, it determines its acceleration reach maximum acceleration or not.

decceleration part is determine by its velocity difference reach a threshold velocity determined by equation 

*v(threshold) =  v(final) - acceleration^2 / 2*max_jerk*


output:

![image](https://github.com/NgChowYong/S_Curve/blob/main/result.png)

# parameter to be set:
-max speed
-max acceleration
-max jerk
-scale (for smoother output, but will cause delay)
-vel_threshold (stable output near desire vel)
-acc_threshold (avoid overshoot of acceleration)


