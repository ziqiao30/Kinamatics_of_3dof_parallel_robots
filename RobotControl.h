#ifndef ROBOTCONTROL_H
#define ROBOTCONTROL_H

#include <Arduino.h>
#include <math.h>
#include <ArduinoEigenDense.h>
using namespace Eigen;
extern void serialMsg(const char* msg);

#define N_MOTOR 3
#define N_SQRT_SOL 2

#define RADIUS_HEXAGON_ENSCRIBED_CIRCLE 0.04401
#define LEG_LENGTH 0.150

// Class to manage the robot's kinematic operations
class RobotControl {
public:
    // Constructor
    RobotControl();
    RobotControl(const Eigen::Vector3d& custom_legconfig, double leg_radius, double half_leg_length);
    
    // Structure to store the reference motor angles
    struct referencemotorangle {
        float motors[N_MOTOR] = { 0.0, 0.0, 0.0 };
        bool valid_solution = true;
        bool multiple_solutions = false;
    };

    struct differentialKinematics {
        Eigen::Vector3d position;
        Eigen::Vector3d velocity;
        Matrix3d Jacobian;
    };

    // Function to compute the inverse kinematics given the desired position
    referencemotorangle Inverse_kinematics(float delta, float psi, float r0);

    // Function to compute the forward kinematics given the motor angles
    Eigen::VectorXd forwardkinematics(Eigen::VectorXd theta);

    // Function to compute the Jacobian of the pose
    differentialKinematics computeDifferentialKinematicsPose(Eigen::VectorXd theta, Eigen::Vector3d theta_dot);
    differentialKinematics computeDifferentialKinematicsCartesian(Eigen::VectorXd theta, Eigen::Vector3d theta_dot);

private:
    // Robot parameters
    double radius;
    double half_leg_length;

    // Configuration for the robot legs
    Eigen::Vector3d legconfig;
};

#endif // ROBOTCONTROL_H
