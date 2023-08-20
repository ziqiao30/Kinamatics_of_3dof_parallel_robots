# RobotControl Library

## About this repository
The RobotControl library is a specialized tool designed for kinematic operations of a robot. It offers an object-oriented approach, encapsulating the essential parameters and functions required for both forward and inverse kinematics.

## Installation and setup

Before using this library, please ensure that the [ArduinoEigenDense](https://github.com/hideakitai/ArduinoEigen) library is correctly installed and integrated into your development environment.

## Member Variables:
- `r`: Represents the distance between the center of the base and the hinge of a leg. 
  - **Default**: `0.04406`
- `l`: Represents the length of a robot leg. 
  - **Default**: `0.0594`
- `legconfig`: A vector storing the configuration of the robot's legs. 
  - **Default**: `{0, 2 * M_PI / 3, 4 * M_PI / 3}`

## Functions


#### **1. Inverse_kinematics**:
  - **Inputs**:
    - `delta`: Desired angle in degrees.
    - `psi`: Desired inclination angle in degrees.
    - `r0`: Desired height or distance of the robot from the ground.
  - **Output**:
    - Returns a `referencemotorangle` structure containing the calculated motor angles for the three motors.
  - **Purpose**:
    - Calculates the motor angles required to achieve a specific position and orientation for the robot.

#### **2. forwardkinematics**:
  - **Inputs**:
    - `theta`: A vector containing the current angles of the motors.
  - **Output**:
    - Returns a vector with five elements. The first three elements represent the robot's position in 3D space, and the last two elements represent its orientation angles.
  - **Purpose**:
    - Computes the position and orientation of the robot in 3D space given the current motor angles.
## **Usage Example**

To utilize the `RobotControl` library, follow the steps below:

1. **Include the Library**:
   ```cpp
   #include "RobotControl.h"
2. **Initialize an Object**:
   ```cpp
   RobotControl robot;
4. **Set Desired Parameters**:
   If the robots configration is different than the default values, then:
   ```cpp
   robot.r = 0.045;
   robot.l = 0.060;
   robot.legconfig << M_PI/4, M_PI/2, 3*M_PI/4;

6. **Compute Inverse Kinematics**:
   ```cpp
   RobotControl::referencemotorangle refAngles = robot.Inverse_kinematics(30.0, 45.0, 0.05);
8. **Compute Forward Kinematics**:
   ```cpp
   Eigen::VectorXd theta(3);
   theta << M_PI/6, M_PI/4, M_PI/3;
   Eigen::VectorXd result = robot.forwardkinematics(theta);
10. **Process Results**:
  Use the results from the kinematic functions in your robotic application as needed.
## Contact person
ziqiao wang (ziqiao.wang@epfl.ch)
