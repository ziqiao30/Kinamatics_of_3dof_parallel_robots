#include "RobotControl.h"

// RobotControl::RobotControl() : r(0.04406), l(0.0594) {
//     legconfig << 0, 2 * M_PI / 3, 4 * M_PI / 3;
// }

RobotControl::RobotControl(const Eigen::Vector3d& custom_legconfig, double leg_radius, double half_leg_length)
    : radius(leg_radius), half_leg_length(half_leg_length) {
    legconfig = custom_legconfig;
}


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

    // Calculate the rotation angle for the top plane's norm vector
    Eigen::Vector3d n_hat = N.normalized();
    if (n_hat.z() < 0){ //Ensure the normal vector is pointing upwards
        n_hat = -n_hat;
        d = -d;
    }

    Eigen::Vector3d pos = 2 * d * n_hat;

    double delta = atan2(n_hat(1), n_hat(0));
    double psi = 2 * asin(sqrt(n_hat(0) * n_hat(0) + n_hat(1) * n_hat(1)));

    result << pos(0), pos(1), pos(2), delta, psi;
    return result;
}

RobotControl::differentialKinematics RobotControl::computeDifferentialKinematicsPose(Eigen::VectorXd theta, Eigen::Vector3d theta_dot){

    //given the position and velocity of the motor angles, compute the rate of change of the pose

    //Compute the position of the joints
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

    Eigen::Vector3d b1_dot, b2_dot, b3_dot;
    b1_dot << -half_leg_length * std::cos(legconfig(0)) * std::sin(theta(0)),
        -half_leg_length * std::sin(legconfig(0)) * std::sin(theta(0)),
         half_leg_length * std::cos(theta(0));
    b2_dot << -half_leg_length * std::cos(legconfig(1)) * std::sin(theta(1)),
        -half_leg_length * std::sin(legconfig(1)) * std::sin(theta(1)),
         half_leg_length * std::cos(theta(1));
    b3_dot << -half_leg_length * std::cos(legconfig(2)) * std::sin(theta(2)),
        -half_leg_length * std::sin(legconfig(2)) * std::sin(theta(2)),
         half_leg_length * std::cos(theta(2));

    Eigen::Vector3d N = (b1 - b2).cross(b1 - b3);
    double Nnorm = N.norm();
    Eigen::Vector3d n_hat = N / Nnorm;
    double sign_flip = 1.0;
    if (n_hat.z() < 0){ //Ensure the normal vector is pointing upwards
        n_hat = -n_hat;
        sign_flip = -1.0;
    }
    double nx = n_hat(0), ny = n_hat(1), nz = n_hat(2);

    double d = b1.dot(n_hat); // Use the (potentially flipped) n_hat for consistency
    double r0 = 2.0 * d;

    double psi   = 2.0 * std::acos(nz);
    double delta = std::atan2(ny, nx);

    // Compute Jacobian
    Matrix3d P = Matrix3d::Identity() - n_hat * n_hat.transpose();

    // Partial derivatives of N
    Vector3d dN1 = b1_dot.cross(b1 - b3) + (b1 - b2).cross(b1_dot);
    Vector3d dN2 = - b2_dot.cross(b1 - b3);
    Vector3d dN3 = - (b1 - b2).cross(b3_dot);

    Vector3d dN[3] = { dN1, dN2, dN3 };

    Matrix3d J;
    J.setZero();

    for (int j = 0; j < 3; ++j)
    {
        // ∂nhat with sign correction if n_hat was flipped
        Vector3d dn = sign_flip * (P * dN[j]) / Nnorm;
        Vector3d db1 = (j == 0 ? b1_dot : Vector3d::Zero());

        // d' (derivative of distance parameter)
        double dd = db1.dot(n_hat) + b1.dot(dn);

        // delta'
        double denomDelta = nx*nx + ny*ny;
        if (denomDelta < 1e-12) denomDelta = 1e-12;
        J(0, j) = (nx * dn(1) - ny * dn(0)) / denomDelta;

        // psi' 
        double denomPsi = std::sqrt(std::max(1e-12, 1.0 - nz*nz));
        J(1, j) = -2.0 * dn(2) / denomPsi;

        // r0' 
        J(2, j) = 2.0 * dd;
    }

    //Pose velocity = J * theta_dot
    Eigen::Vector3d pose_velocity = J * theta_dot;

    differentialKinematics out;
    out.position << delta, psi, r0;
    out.velocity = pose_velocity;
    out.Jacobian = J;

    return out;

}


RobotControl::differentialKinematics RobotControl::computeDifferentialKinematicsCartesian(Eigen::VectorXd theta, Eigen::Vector3d theta_dot){

    //given the position and velocity of the motor angles, compute the rate of change of the cartesian position position of the centre of the end effector

    //Compute the position of the joints
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

    Eigen::Vector3d b1_dot, b2_dot, b3_dot;
    b1_dot << -half_leg_length * std::cos(legconfig(0)) * std::sin(theta(0)),
        -half_leg_length * std::sin(legconfig(0)) * std::sin(theta(0)),
         half_leg_length * std::cos(theta(0));
    b2_dot << -half_leg_length * std::cos(legconfig(1)) * std::sin(theta(1)),
        -half_leg_length * std::sin(legconfig(1)) * std::sin(theta(1)),
         half_leg_length * std::cos(theta(1));
    b3_dot << -half_leg_length * std::cos(legconfig(2)) * std::sin(theta(2)),
        -half_leg_length * std::sin(legconfig(2)) * std::sin(theta(2)),
         half_leg_length * std::cos(theta(2));

    Eigen::Vector3d N = (b1 - b2).cross(b1 - b3);
    double Nnorm = N.norm();
    Eigen::Vector3d n_hat = N / Nnorm;
    double sign_flip = 1.0;
    if (n_hat.z() < 0){ //Ensure the normal vector is pointing upwards
        n_hat = -n_hat;
        sign_flip = -1.0;
    }
    double nx = n_hat(0), ny = n_hat(1), nz = n_hat(2);

    double d = b1.dot(n_hat); // Use the (potentially flipped) n_hat for consistency
    double r0 = 2.0 * d;
    Eigen::Vector3d position = 2.0 * d * n_hat; //position of the centre of the end effector

    // Compute Jacobian
    Matrix3d P = Matrix3d::Identity() - n_hat * n_hat.transpose();

    // Partial derivatives of N
    Vector3d dN1 = b1_dot.cross(b1 - b3) + (b1 - b2).cross(b1_dot);
    Vector3d dN2 = - b2_dot.cross(b1 - b3);
    Vector3d dN3 = - (b1 - b2).cross(b3_dot);

    Vector3d dN[3] = { dN1, dN2, dN3 };

    Matrix3d J_xyz = Matrix3d::Zero();
    for (int j = 0; j < 3; ++j) {
        // ∂nhat = (1/N) * P * ∂N, with sign correction if n_hat was flipped
        Vector3d dnhat = sign_flip * (P * dN[j]) / Nnorm;

        // ∂d = (∂b1)·nhat + b1·(∂nhat)
        Vector3d db1 = (j == 0 ? b1_dot : Vector3d::Zero());
        double dd = db1.dot(n_hat) + b1.dot(dnhat);

        // ∂p = 2*(dd)*nhat + 2*d * dnhat
        Vector3d dp_dth = 2.0 * dd * n_hat + 2.0 * d * dnhat;

        J_xyz.col(j) = dp_dth;
    }

    //Cartesian velocity = J_xyz * theta_dot
    Eigen::Vector3d cartesian_velocity = J_xyz * theta_dot;

    differentialKinematics out;
    out.position = position;
    out.velocity = cartesian_velocity;
    out.Jacobian = J_xyz;

    return out;

}
