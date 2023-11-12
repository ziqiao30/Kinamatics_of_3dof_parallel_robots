#ifndef ROBOTCONTROL_H
#define ROBOTCONTROL_H

#include <Arduino.h>
#include <math.h>
#include <ArduinoEigenDense.h>
using namespace Eigen;


// Class to manage the robot's kinematic operations
class RobotControl {
public:
    // Constructor
    RobotControl();
    
    // Structure to store the reference motor angles
    struct referencemotorangle {
        uint16_t motor1;
        uint16_t motor2;
        uint16_t motor3;
    };

    // Function to compute the inverse kinematics given the desired position
    referencemotorangle Inverse_kinematics(float delta, float psi, float r0);

    // Function to compute the forward kinematics given the motor angles
    Eigen::VectorXd forwardkinematics(Eigen::VectorXd theta);

private:
    // Robot parameters
    double r, l;

    // Configuration for the robot legs
    Eigen::Vector3d legconfig;
};

#endif // ROBOTCONTROL_H
