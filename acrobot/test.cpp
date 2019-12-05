#include <iostream>

#include <Eigen/Dense>

#include <iit/robots/acrobot/declarations.h>
#include <iit/robots/acrobot/transforms.h>
#include <iit/robots/acrobot/jacobians.h>


int main(int argc, char** argv) {

    // joint states are just Vector2d
    iit::Acrobot::JointState q;
    iit::Acrobot::JointState dq;
    iit::Acrobot::JointState dqq;
    iit::Acrobot::JointState tau;



    // iit::Acrobot::MotionTransforms xm;
    iit::Acrobot::HomogeneousTransforms xm;
    iit::Acrobot::Jacobians J;
    

    // these was tested against my acrobot_rod.nb
    q = Eigen::Vector2d(0,0);
    std::cout << "0,0  - "<< std::endl << xm.fr_base0_X_ee(q) << std::endl;
    q = Eigen::Vector2d(M_PI/3,0);
    std::cout << "M_PI/3,0  - " << std::endl << xm.fr_base0_X_ee(q) << std::endl;
    q = Eigen::Vector2d(M_PI/3, M_PI/2);
    std::cout << "M_PI/3,M_PI/2  - " << std::endl << xm.fr_base0_X_ee(q) << std::endl;


    q = Eigen::Vector2d(M_PI/3, M_PI/2);
    dq = Eigen::Vector2d(M_PI/3, M_PI/3);
    std::cout << "Jacobian  - " << std::endl << J.fr_base0_J_ee(q) << std::endl;

    // according to acrobot_rod.nb test, this J is analytical jacobian
    Eigen::VectorXd ee_vel_body = J.fr_base0_J_ee(q)*dq;
    std::cout << "Jacobian*dq  - " << std::endl << ee_vel_body << std::endl;
    
    return 0;
}

