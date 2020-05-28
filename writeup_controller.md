## Project: Building a Controller

# Required Steps for a Passing Submission:
1. Body rate and roll/pitch control (scenario 2)
   - *GenerateMotorCommands()*
   - *BodyRateControl()*
   - *RollPitchControl()*
2. Position/velocity and yaw angle control (scenario 3)
   - *LateralPositionControl()*
   - *AltitudeControl()*
   - *YawControl()*
3. Non-idealities and robustness (scenario 4)
   - adapt *AltitudeControl()*
4. Tracking trajectories (scenario 5)


## [Rubric](https://review.udacity.com/#!/rubrics/1643/view) Points

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf. 

You're reading it! Below I describe how I addressed each rubric point and where in my code each point is handled.

#### 2. Implemented body rate control in C++.
P-controller for the drones body rates (body frame). Implementation is following *body_rate_control(...)* from https://github.com/udacity/FCND-Controls/blob/solution/controller.py 

#### 3. Implement roll pitch control in C++.
P-Controller for the drones roll and pitch rate (body frame). Implementation is following *roll_pitch_controller(...)* from https://github.com/udacity/FCND-Controls/blob/solution/controller.py based on formulas presented in 3D Drone Exercise 4.2.

#### 4. Implement altitude controller in C++.
In the first version this is a PD controller to provide the drones thrust needed to reach the target z-position and target vertical velocity. The implementation is following the formulas presented in 3D Drone Exercise 5.3. Additionally, the vertical acceleration is clipped between [-maxAscentRate/dt, maxDescentRate/dt] and the calculated vertical acceleration is converted to thrust. In order to pass scenario 4 an integrator had to be added, compensating the systematic bias caused by the different drone masses.

#### 5. Implement lateral position control in C++.
PD-controller for the drones x-y-acceleration needed to reach the provided x-y-position and velocity. Implementation is following *lateral_position_control(...)* from https://github.com/udacity/FCND-Controls/blob/solution/controller.py based on formulas presented in 3D Drone Exercise 4.1. The differences are:

- Instead of calculating rotation matrix elements as done in 3D Drone Exercise 4.1, this implementation is calculating the x-y-acceleration, which is directly provided to the roll pitch controller implemented in 3. 
- Furthermore, the calculated horizontal acceleration is limited to *maxAccelXY*.
- In the python file the method *trajectory_control(...)* calculates the target velocity by simply taking the quotient over the position and time delta, thus the resulting value can easily exceed *maxSpeedXY*. It is assumed that this is the same for the C++ code, therefore the horizontal target velocity has to be directly limited to *maxSpeedXY* before calculating the respective p-term.

#### 6. Implement yaw control in C++.
P-controller for the drones yaw rate needed to reach given yaw rate. Implementation is following *yaw_control(...)* from https://github.com/udacity/FCND-Controls/blob/solution/controller.py based on formulas presented in 3D Drone Exercise 5.2.

#### 7. Implement calculating the motor commands given commanded thrust and moments in C++.
Taken equations given 3D Drone Exercise and set up linear equation system with tau_x, tau_y, tau_z, c, dependent on F_i with i = 1,...,4, the arm length L, k_m  and k_f with kappa = k_m/k_f. The equations were solved with a linear equation solver. After having a closer look to the drones rotor numbering provided at top of the 3D Drone Exercise, Force 3 and 4 had to be switched for the given quadcopter.

#### 8. Your C++ controller is successfully able to fly the provided test trajectory and visually passes inspection of the scenarios leading up to the test trajectory.
After another round of tuning, all scenarios work and pass the evaluations!