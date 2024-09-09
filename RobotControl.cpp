#include "RobotControl.h"

RobotControl::RobotControl() : r(0.04406), l(0.0594) {
    legconfig << 0, 2 * M_PI / 3, 4 * M_PI / 3;
}

RobotControl::referencemotorangle RobotControl::Inverse_kinematics(float delta, float psi, float r0) {
    referencemotorangle refangle;
    const double eps = 1e-8;

    //Conversion removed. Inverse kinemeatics now expects Rad
    
    // Adjust psi and r0 values
    psi /= 2;
    r0 /= sin(M_PI / 2 - psi);
    
    // Define angle limits and other constants
    double lower_limits[3] = {M_PI_2 / 3, M_PI_2 / 3, M_PI_2 / 3};
    double upper_limits[3] = {M_PI_2, M_PI_2, M_PI_2};
    double sols[2][3];
    double phi[2] = {0, 0};
    bool valid_sol[2]= {false};

    // Calculate motor angles
    for (int i = 0; i < 3; i++) {
        double a = (r - l) * (sin(psi) * cos(delta - legconfig(i))) - (r0 / 2);
        double b = 2 * l * cos(psi);
        double c = (r + l) * (sin(psi) * cos(delta - legconfig(i))) - (r0 / 2);
        double root = b * b - 4 * a * c;
        
        if (root >= 0) {
            phi[0] = (-b + sqrt(root)) / (2 * a);
            phi[1] = (-b - sqrt(root)) / (2 * a);
            for (int j = 0; j < 2; j++) {
                phi[j] = 2 * atan2(phi[j], 1);
                if (phi[j] > lower_limits[i] - eps && phi[j] < upper_limits[i] + eps) {
                    sols[j][i] = (phi[j] * 180) / M_PI;
                    valid_sol[j] = true;
                }
            }
        } 
        else {
            serialMsg("Inverse Kinematics.Error - no real solution for current reference workpoint");
            refangle.valid_solution = false;
        }
    }
    if((valid_sol[0] == valid_sol[1]) && (valid_sol[0] == true)){
        refangle.multiple_solutions = true;
    }
    for(int s = 0; s < sizeof(valid_sol)/sizeof(valid_sol[0]); s++){
        if(valid_sol[s] == true){
            refangle.motor1 = sols[s][0] * M_PI /180;
            refangle.motor2 = sols[s][1] * M_PI /180;
            refangle.motor3 = sols[s][2] * M_PI /180;
            refangle.valid_solution = true;
        }
    }

    //Returns angles in radians
    return refangle;
}

Eigen::VectorXd RobotControl::forwardkinematics(Eigen::VectorXd theta) {
    Eigen::VectorXd result(5);

    // Define constants for calculation
    Eigen::Vector3d b1, b2, b3;
    b1 << cos(legconfig(0)) * (r + l * cos(theta(0))),
          sin(legconfig(0)) * (r + l * cos(theta(0))), 
          l * sin(theta(0));
    b2 << cos(legconfig(1)) * (r + l * cos(theta(1))),
          sin(legconfig(1)) * (r + l * cos(theta(1))), 
          l * sin(theta(1));
    b3 << cos(legconfig(2)) * (r + l * cos(theta(2))),
          sin(legconfig(2)) * (r + l * cos(theta(2))), 
          l * sin(theta(2));

    // Compute normal vector (N) and its distance (d)
    Eigen::Vector3d N = (b1 - b2).cross(b1 - b3);
    double d = (b1 - Eigen::Vector3d::Zero()).dot(N) / N.norm();
    Eigen::Vector3d pos = 2 * d * N / N.norm();

    // Calculate the rotation angle for the top plane's norm vector
    N.normalize();
    double alpha = atan2(N(1), N(0)) + M_PI;
    double beta = 2 * asin(sqrt(N(0) * N(0) + N(1) * N(1)));

    result << pos(0), pos(1), pos(2), alpha, beta;
    return result;
}
