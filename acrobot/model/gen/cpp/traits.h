#ifndef IIT_ROBOGEN__ACROBOT_TRAITS_H_
#define IIT_ROBOGEN__ACROBOT_TRAITS_H_

#include "declarations.h"
#include "transforms.h"
#include "inverse_dynamics.h"
#include "forward_dynamics.h"
#include "jsim.h"
#include "inertia_properties.h"

namespace iit {
namespace Acrobot {

struct Traits {
    typedef typename Acrobot::ScalarTraits ScalarTraits;

    typedef typename Acrobot::JointState JointState;

    typedef typename Acrobot::JointIdentifiers JointID;
    typedef typename Acrobot::LinkIdentifiers  LinkID;

    typedef typename Acrobot::HomogeneousTransforms HomogeneousTransforms;
    typedef typename Acrobot::MotionTransforms MotionTransforms;
    typedef typename Acrobot::ForceTransforms ForceTransforms;

    typedef typename Acrobot::dyn::InertiaProperties InertiaProperties;
    typedef typename Acrobot::dyn::ForwardDynamics FwdDynEngine;
    typedef typename Acrobot::dyn::InverseDynamics InvDynEngine;
    typedef typename Acrobot::dyn::JSIM JSIM;

    static const int joints_count = Acrobot::jointsCount;
    static const int links_count  = Acrobot::linksCount;
    static const bool floating_base = false;

    static inline const JointID* orderedJointIDs();
    static inline const LinkID*  orderedLinkIDs();
};


inline const Traits::JointID*  Traits::orderedJointIDs() {
    return Acrobot::orderedJointIDs;
}
inline const Traits::LinkID*  Traits::orderedLinkIDs() {
    return Acrobot::orderedLinkIDs;
}

}
}

#endif
