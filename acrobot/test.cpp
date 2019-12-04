#include <iostream>

#include <Eigen/Dense>

#include <iit/robots/acrobot/declarations.h>
#include <iit/robots/acrobot/transforms.h>


int main(int argc, char** argv) {

    iit::Acrobot::MotionTransforms xm;
    iit::Acrobot::JointState q;

    q = Eigen::Vector2d(M_PI,0.0);

    std::cout << xm.fr_base0_X_ee(q) << std::endl;
    
    return 0;
}

