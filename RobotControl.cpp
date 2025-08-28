#include "RobotControl.h"

// RobotControl::RobotControl() : r(0.04406), l(0.0594) {
//     legconfig << 0, 2 * M_PI / 3, 4 * M_PI / 3;
// }

RobotControl::RobotControl() : radius(RADIUS_HEXAGON_ENSCRIBED_CIRCLE), half_leg_length(LEG_LENGTH / 2) {
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
    double lower_limits[N_MOTOR] = {0, 0, 0};
    double upper_limits[N_MOTOR] = {M_PI_2, M_PI_2, M_PI_2};
    double sols[N_MOTOR][N_SQRT_SOL];
    double phi[N_SQRT_SOL] = {0,0};
    bool valid_sol[N_MOTOR][N_SQRT_SOL] = {{false,false},{false,false},{false,false}};

    // Calculate motor angles
    for (int i = 0; i < N_MOTOR; i++) {
        double a = (radius - half_leg_length) * (sin(psi) * cos(delta - legconfig(i))) - (r0 / 2);
        double b = 2 * half_leg_length * cos(psi);
        double c = (radius + half_leg_length) * (sin(psi) * cos(delta - legconfig(i))) - (r0 / 2);
        double root = b * b - 4 * a * c;
        
        if (root >= 0) {
            phi[0] = (-b + sqrt(root)) / (2 * a);
            phi[1] = (-b - sqrt(root)) / (2 * a);
            for (int j = 0; j < N_SQRT_SOL; j++){
                phi[j] = 2 * atan2(phi[j], 1);
                if(phi[j] > lower_limits[i] - eps && phi[j] < upper_limits[i] + eps){
                    sols[i][j] = phi[j];
                    valid_sol[i][j] = true;
                }
            }
        } 
        else {
            //serialMsg("Inverse Kinematics.Error - no real solution for current reference workpoint");
            refangle.valid_solution = false;
        }
        if((valid_sol[i][0]) && (valid_sol[i][1])){
            refangle.multiple_solutions = true;
        }
        if((!valid_sol[i][0]) && (!valid_sol[i][1])){
            refangle.valid_solution = false;
        }
    }

    if(refangle.valid_solution){
        if(refangle.multiple_solutions){
            for(int motor =0; motor< N_MOTOR; motor ++){
                char log_message_buffer [500];
                sprintf(log_message_buffer, "mutliple real solutions found for Motor:%i Delta:%f, Psi:%f, R0:%f Solutions:%f,  %f\n", motor, delta*RAD_TO_DEG, psi*RAD_TO_DEG, r0, sols[motor][0] * RAD_TO_DEG, sols[motor][1] * RAD_TO_DEG);
                serialMsg(log_message_buffer);
            }
        }
    }

    for (int i = 0; i < N_MOTOR; i++) {
        for(int s = 0; s < N_SQRT_SOL; s++){
            if(valid_sol[i][s] == true){
                refangle.motors[i] = sols[i][s];
            }
        }
    }

    //Returns angles in radians
    return refangle;
}

Eigen::VectorXd RobotControl::forwardkinematics(Eigen::VectorXd theta) {
    Eigen::VectorXd result(5);

    // Define constants for calculation
    Eigen::Vector3d b1, b2, b3;
    b1 << cos(legconfig(0)) * (radius + half_leg_length * cos(theta(0))),
          sin(legconfig(0)) * (radius + half_leg_length * cos(theta(0))), 
          half_leg_length * sin(theta(0));
    b2 << cos(legconfig(1)) * (radius + half_leg_length * cos(theta(1))),
          sin(legconfig(1)) * (radius + half_leg_length * cos(theta(1))), 
          half_leg_length * sin(theta(1));
    b3 << cos(legconfig(2)) * (radius + half_leg_length * cos(theta(2))),
          sin(legconfig(2)) * (radius + half_leg_length * cos(theta(2))), 
          half_leg_length * sin(theta(2));

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
