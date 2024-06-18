#ifndef ROBOTCONTROL_H
#define ROBOTCONTROL_H

#include <Arduino.h>
#include <math.h>
#include <ArduinoEigenDense.h>
using namespace Eigen;
extern void serialMsg(const char * msg);


// Class to manage the robot's kinematic operations
class RobotControl {
public:
    // Constructor
    RobotControl();
    
    // Structure to store the reference motor angles
    struct referencemotorangle {
        float motor1; //Changed from uint16_t
        float motor2;
        float motor3;
        bool valid_solution;
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
