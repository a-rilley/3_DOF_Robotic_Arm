# 3_DOF_Robotic_Arm
## Robotic Arm Simulation in MATLAB Simulink with Arduino prototype
This project involved the development of a 3-degree of freedom robotic arm / manipulator. The MATLAB Simulink solution and the physical prototype were critically damped, if not slightly overdamped. However, for the short movements it needed to perform, this was ok at the time. It could be improved by properly tuning the PID controller.

## Project Description
Design and simulation of a mechatronic system which actuates a robotic arm using DC motors with hall-effect encoders. Servo motors were prohibited from being used for joint actuation as the purpose of the project was to combine actuators and power electronics. PID control for motor speed and position. The home position of each motor was set using magnets and hall-effect sensors. The position of the end-effector was calculated using forward and inverse kinematics. Actuation of the joints was simulated using MATLAB Simulink and Simscape. The prototype was designed in Solidworks, 3D printed, and assembled to demonstrate functionality.

- Arduino, DC motors, L298N dual H-bridge motor driver
- PID Control: speed, position
- MATLAB: Simulink, Simscape

## Simulink Model
Full Model
![alt text](https://github.com/a-rilley/3_DOF_Robotic_Arm/blob/main/Simulink%20Blocks/Simulink%20Model.png)

Motor Block
![alt text](https://github.com/a-rilley/3_DOF_Robotic_Arm/blob/main/Simulink%20Blocks/Simulink%20Model-Motor%20Block.png)

## Prototype
The CAD design of the prototype was a modified version of Dejan Nedelkovski's 4-DOF robotic arm which relied on servo motors for actuation. The design was modified to support 3 high torque, low speed DC motors with hall-effect encoders.

![alt text](https://github.com/a-rilley/3_DOF_Robotic_Arm/blob/main/Photos/Robot%20Arm%20Assembly.png)


Note: This project was completed in early 2021 and there are no plans to make any changes to the core files.
