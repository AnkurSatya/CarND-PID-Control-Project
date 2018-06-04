
## Implementation


The PID implementation was done as taught in the class. The aim was to reduce the cross track error(cte) by reducing the the PID errors.
1. P - Proportional error.
It directly depends upon the cte.
2. I - Integral error.
It is proportional to the sum of the past cross track errors.
3. D - Derivative error.
It depends upon the difference between the present and the immediate past error.

The steering angle is calculated by multiplying the PID coefficients with their respective errors.
So, the task is to optimize these coefficients.

## Effect of P,I, and D components

1. P Component.
The P component tries to bring back the vehicle to the central position by just relying on the cte. If the cte is large, the counter measure given by the P component might result in the overshooting of the center line.
2. I component. 
The use of the I component is to remove any internal bias in the vehicle. The I component did not have any significant effect in my PID implmentation which makes sense as we are running a vehicle in a simulator which is basically bias free, until added explicitly.
3. D component.
The D component tries to the prevent the overshooting by penalising a large change in the cte in a short amount of time. It might happen due to the counter measure given by the P component. It has a smoothning effect on the vehicle movement.

## Choosing the Final Hyperparameters.

The final hyperparameters were chosen in three steps:
1. Twiddle.
I ran the twiddle algorithm until the sum of the P and D coefficients, Kp and Kd respectively, were not below a threshold value of 0.01. This resulted in the following values
Kp = 1.1, Kd = -0.47 
Although these values resulted in a complete lap without any error but the sign of the Kd was not correct.

2. Twiddle Again
So, then I changed the value of Dp(change in value of Kp in twiddle algorithm) to zero which meant that only Kd was being optimized.
This resulted in the following value:
Kp = 1.1, Kd = 0.25

3. Manual Tuning
There was still room for improvement. I manually tuned the value of Kd to 1.25 and the performance improved.
