#include <iit/rbd/utils.h>
#include "miscellaneous.h"

using namespace iit::Acrobot;
using namespace iit::Acrobot::dyn;

Vector3 iit::Acrobot::getWholeBodyCOM(
    const InertiaProperties& inertiaProps,
    const HomogeneousTransforms& ht)
{
    Vector3 tmpSum(Vector3::Zero());


    HomogeneousTransforms::MatrixType tmpX(HomogeneousTransforms::MatrixType::Identity());
    tmpX = tmpX * ht.fr_base0_X_fr_link1;
    tmpSum += inertiaProps.getMass_link1() *
            ( iit::rbd::Utils::transform(tmpX, inertiaProps.getCOM_link1()));
    
    tmpX = tmpX * ht.fr_link1_X_fr_link2;
    tmpSum += inertiaProps.getMass_link2() *
            ( iit::rbd::Utils::transform(tmpX, inertiaProps.getCOM_link2()));
    

    return tmpSum / inertiaProps.getTotalMass();
}

Vector3 iit::Acrobot::getWholeBodyCOM(
    const InertiaProperties& inertiaProps,
    const JointState& q,
    HomogeneousTransforms& ht)
{
    // First updates the coordinate transforms that will be used by the routine
    ht.fr_base0_X_fr_link1(q);
    ht.fr_link1_X_fr_link2(q);

    // The actual calculus
    return getWholeBodyCOM(inertiaProps, ht);
}
