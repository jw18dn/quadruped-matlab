# quadruped-matlab
2 DOF Matlab version of the 2018 MIT Cheetah Paper
Copyright (c) 2023, Jason White

To run the simulation simply open "SimController" and press run.

This simulates a 2 DOF version of MIT cheetah 3. The controller utilizes operational space control
and model predictive control. The simulation runs at 100Hz using ode45 and the full nonlinear version 
of the dynamics. Contact is modeled using holonomic constraints with the impact modeling found in 
"Feedback Control of a Cassie Bipedal Robot: Walking, Standing, and Riding a Segway"

Sytem parameters are saved as a struct and can be modified in the RobotProperties.m file,
any time these a modified the function generators for the kinematics and dynamics must be rerun. 
This is done by uncommenting lines 125-127

Initial conditions and optimization parameters can be modified form line 40-80.
