# 3_DOF_Robotic_Arm
## Robotic Arm Simulation in MATLAB with Arduino prototype
This project involved the development of a 3-degree of freedom robotic arm / manipulator.The MATLAB Simulink solution and the physical prototype were critically damped, if not slightly overdamped, however for the short movements it needed to perform, this was ok at the time. It could be improved by properly tuning the PID controller.

## Project Description
Design and simulation of a mechatronic system which actuates a robotic arm using DC motors with hall-effect encoders. Servo motors were prohibited from being used for joint actuation as the purpose of the project was to combine actuators and power electronics. PID control for motor speed and position. The home position of each motor was set using magnets and hall-effect sensors. The position of the end-effector was calculated using forward and inverse kinematics. Actuation of the joints was simulated using MATLAB Simulink and Simscape. The prototype was designed in Solidworks, 3D printed, and assembled to demonstrate functionality.

- Arduino, DC motors, L298N dual H-bridge motor driver
- PID Control: speed, position
- MATLAB: Simulink, Simscape
