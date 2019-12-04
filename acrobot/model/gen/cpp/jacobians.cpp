#include "jacobians.h"

iit::Acrobot::Jacobians::Jacobians()
:    
    fr_base0_J_ee()
{}

void iit::Acrobot::Jacobians::updateParameters(const Params_lengths& _lengths, const Params_angles& _angles)
{
}

iit::Acrobot::Jacobians::Type_fr_base0_J_ee::Type_fr_base0_J_ee()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 1.0;
    (*this)(4,1) = 0.0;
    (*this)(5,1) = 0.0;
}

const iit::Acrobot::Jacobians::Type_fr_base0_J_ee& iit::Acrobot::Jacobians::Type_fr_base0_J_ee::update(const JointState& q)
{
    Scalar sin_q_jA  = ScalarTraits::sin( q(JA) );
    Scalar cos_q_jA  = ScalarTraits::cos( q(JA) );
    (*this)(4,0) = ( tx_jB+ tx_ee) * cos_q_jA;
    (*this)(5,0) = ( tx_jB+ tx_ee) * sin_q_jA;
    return *this;
}

