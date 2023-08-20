# RobotControl Library

## About this repository
The RobotControl library is a specialized tool designed for kinematic operations of a robot. It offers an object-oriented approach, encapsulating the essential parameters and functions required for both forward and inverse kinematics.

## Installation and setup

Before using this library, please ensure that the [ArduinoEigenDense](https://github.com/hideakitai/ArduinoEigen) library is correctly installed and integrated into your development environment.

## Member Variables:
r: Represents the distance between the center of the base and the hinge of a leg. Default value is 0.04406.
l: Represents the length of a robot leg. Default value is 0.0594.
legconfig: A vector storing the configuration of the robot's legs. Default values are {0, 2 * M_PI / 3, 4 * M_PI / 3}.

## Functions


### Forward Kinematics
Inputs:
theta: A vector containing the current angles of the motors.
Output:
Returns a vector with five elements. The first three elements represent the robot's position in 3D space, and the last two elements represent its orientation angles.
Purpose:
Given the current motor angles, this function computes the position and orientation of the robot in 3D space.
  
### Inverse Kinematics
Inputs:
delta: Desired angle in degrees.
psi: Desired inclination angle in degrees.
r0: Desired height or distance of the robot from the ground.
Output:
Returns a referencemotorangle structure containing the calculated motor angles for the three motors.
Purpose:
This function calculates the motor angles required to achieve a specific position and orientation for the robot.

## Contact person
