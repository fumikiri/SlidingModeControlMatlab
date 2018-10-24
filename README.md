# SlidingModeControlMatlab

## Overveiw
This repository is for simulation code of sliding mode control (SMC) in Matlab/Simulink.
Plant model is Inverted pendulum( it's model is linearlized) and sliding mode control is used to make it's dynamics stable.
Sliding mode control is compared to Linear quadratic (LQ) control in this simulation.

## Comment
Sliding mode control is one of the strong robust control method in control theory, but it's theory is sometimes difficult to understand and construct simulation code.
I hope this simulation become someone help.

## Animation
![result](https://github.com/fumikiri/SlidingModeControlMatlab/tree/master/GIF/lq_vs_smc_animation.gif)

Each Animation is shown this link.
Inverted pendulum has nonlinear element(dead zone) and time response of linear quadratic control has vibration around 0.
On the other hand, time response of sliding mode controller has less vibration even though dead zone exists.

