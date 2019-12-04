#include "transforms.h"
#include "jsim.h"

#include <iit/rbd/robcogen_commons.h>

using namespace iit::rbd;

//Implementation of default constructor
iit::Acrobot::dyn::JSIM::JSIM(InertiaProperties& inertiaProperties, ForceTransforms& forceTransforms) :
    linkInertias(inertiaProperties),
    frcTransf( &forceTransforms ),
    link2_Ic(linkInertias.getTensor_link2())
{
    //Initialize the matrix itself
    this->setZero();
}

#define DATA operator()
const iit::Acrobot::dyn::JSIM& iit::Acrobot::dyn::JSIM::update(const JointState& state) {
    Force F;

    // Precomputes only once the coordinate transforms:
    frcTransf -> fr_link1_X_fr_link2(state);

    // Initializes the composite inertia tensors
    link1_Ic = linkInertias.getTensor_link1();

    // "Bottom-up" loop to update the inertia-composite property of each link, for the current configuration

    // Link link2:
    iit::rbd::transformInertia<Scalar>(link2_Ic, frcTransf -> fr_link1_X_fr_link2, Ic_spare);
    link1_Ic += Ic_spare;

    F = link2_Ic.col(LZ);
    DATA(JB, JB) = F(LZ);

    F = frcTransf -> fr_link1_X_fr_link2 * F;
    DATA(JB, JA) = F(AZ);
    DATA(JA, JB) = DATA(JB, JA);

    // Link link1:

    F = link1_Ic.col(AZ);
    DATA(JA, JA) = F(AZ);


    return *this;
}

#undef DATA
#undef F

void iit::Acrobot::dyn::JSIM::computeL() {
    L = this -> triangularView<Eigen::Lower>();
    // Joint jB, index 4 :
    L(4, 4) = ScalarTraits::sqrt(L(4, 4));
    L(4, 0) = L(4, 0) / L(4, 4);
    L(0, 0) = L(0, 0) - L(4, 0) * L(4, 0);
    
    // Joint jA, index 0 :
    L(0, 0) = ScalarTraits::sqrt(L(0, 0));
    
}

void iit::Acrobot::dyn::JSIM::computeInverse() {
    computeLInverse();

    inverse(0, 0) =  + (Linv(0, 0) * Linv(0, 0));
    inverse(4, 4) =  + (Linv(4, 0) * Linv(4, 0)) + (Linv(4, 4) * Linv(4, 4));
    inverse(4, 0) =  + (Linv(4, 0) * Linv(0, 0));
    inverse(0, 4) = inverse(4, 0);
}

void iit::Acrobot::dyn::JSIM::computeLInverse() {
    //assumes L has been computed already
    Linv(0, 0) = 1 / L(0, 0);
    Linv(4, 4) = 1 / L(4, 4);
    Linv(4, 0) = - Linv(0, 0) * ((Linv(4, 4) * L(4, 0)) + 0);
}
