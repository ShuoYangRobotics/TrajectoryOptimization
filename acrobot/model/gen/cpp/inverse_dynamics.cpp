#include <iit/rbd/robcogen_commons.h>

#include "inverse_dynamics.h"
#include "inertia_properties.h"
#ifndef EIGEN_NO_DEBUG
    #include <iostream>
#endif
using namespace std;
using namespace iit::rbd;
using namespace iit::Acrobot::dyn;

// Initialization of static-const data
const iit::Acrobot::dyn::InverseDynamics::ExtForces
iit::Acrobot::dyn::InverseDynamics::zeroExtForces(Force::Zero());

iit::Acrobot::dyn::InverseDynamics::InverseDynamics(InertiaProperties& inertia, MotionTransforms& transforms) :
    inertiaProps( & inertia ),
    xm( & transforms ),
    link1_I(inertiaProps->getTensor_link1() ),
    link2_I(inertiaProps->getTensor_link2() )
    {
#ifndef EIGEN_NO_DEBUG
    std::cout << "Robot Acrobot, InverseDynamics::InverseDynamics()" << std::endl;
    std::cout << "Compiled with Eigen debug active" << std::endl;
#endif
    link1_v.setZero();
    link2_v.setZero();

    vcross.setZero();
}

void iit::Acrobot::dyn::InverseDynamics::id(
    JointState& jForces,
    const JointState& qd, const JointState& qdd,
    const ExtForces& fext)
{
    firstPass(qd, qdd, fext);
    secondPass(jForces);
}

void iit::Acrobot::dyn::InverseDynamics::G_terms(JointState& jForces)
{
    // Link 'link1'
    link1_a = (xm->fr_link1_X_fr_base0).col(iit::rbd::LZ) * Acrobot::g;
    link1_f = link1_I * link1_a;
    // Link 'link2'
    link2_a = (xm->fr_link2_X_fr_link1) * link1_a;
    link2_f = link2_I * link2_a;

    secondPass(jForces);
}

void iit::Acrobot::dyn::InverseDynamics::C_terms(JointState& jForces, const JointState& qd)
{
    // Link 'link1'
    link1_v(iit::rbd::AZ) = qd(JA);   // link1_v = vJ, for the first link of a fixed base robot
    
    link1_f = vxIv(qd(JA), link1_I);
    
    // Link 'link2'
    link2_v = ((xm->fr_link2_X_fr_link1) * link1_v);
    link2_v(iit::rbd::LZ) += qd(JB);
    
    motionCrossProductMx<Scalar>(link2_v, vcross);
    
    link2_a = (vcross.col(iit::rbd::LZ) * qd(JB));
    
    link2_f = link2_I * link2_a + vxIv(link2_v, link2_I);
    

    secondPass(jForces);
}


void iit::Acrobot::dyn::InverseDynamics::firstPass(const JointState& qd, const JointState& qdd, const ExtForces& fext)
{
    // First pass, link 'link1'
    link1_a = (xm->fr_link1_X_fr_base0).col(iit::rbd::LZ) * Acrobot::g;
    link1_a(iit::rbd::AZ) += qdd(JA);
    link1_v(iit::rbd::AZ) = qd(JA);   // link1_v = vJ, for the first link of a fixed base robot
    
    link1_f = link1_I * link1_a + vxIv(qd(JA), link1_I)  - fext[LINK1];
    
    // First pass, link 'link2'
    link2_v = ((xm->fr_link2_X_fr_link1) * link1_v);
    link2_v(iit::rbd::LZ) += qd(JB);
    
    motionCrossProductMx<Scalar>(link2_v, vcross);
    
    link2_a = (xm->fr_link2_X_fr_link1) * link1_a + vcross.col(iit::rbd::LZ) * qd(JB);
    link2_a(iit::rbd::LZ) += qdd(JB);
    
    link2_f = link2_I * link2_a + vxIv(link2_v, link2_I) - fext[LINK2];
    
}

void iit::Acrobot::dyn::InverseDynamics::secondPass(JointState& jForces)
{
    // Link 'link2'
    jForces(JB) = link2_f(iit::rbd::LZ);
    link1_f += xm->fr_link2_X_fr_link1.transpose() * link2_f;
    // Link 'link1'
    jForces(JA) = link1_f(iit::rbd::AZ);
}
