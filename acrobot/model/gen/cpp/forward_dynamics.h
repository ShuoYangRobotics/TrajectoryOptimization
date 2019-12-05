#ifndef IIT_ROBOT_ACROBOT_FORWARD_DYNAMICS_H_
#define IIT_ROBOT_ACROBOT_FORWARD_DYNAMICS_H_

#include <iit/rbd/rbd.h>
#include <iit/rbd/InertiaMatrix.h>
#include <iit/rbd/utils.h>

#include "declarations.h"
#include "transforms.h"
#include "inertia_properties.h"
#include "link_data_map.h"

namespace iit {
namespace Acrobot {
namespace dyn {

/**
 * The Forward Dynamics routine for the robot Acrobot.
 *
 * The parameters common to most of the methods are the joint status \c q, the
 * joint velocities \c qd and the joint forces \c tau. The accelerations \c qdd
 * will be filled with the computed values. Overloaded methods without the \c q
 * parameter use the current configuration of the robot; they are provided for
 * the sake of efficiency, in case the kinematics transforms of the robot have
 * already been updated elsewhere with the most recent configuration (eg by a
 * call to setJointStatus()), so that it would be useless to compute them again.
 */
class ForwardDynamics {
public:
    typedef LinkDataMap<Force> ExtForces;

    /**
     * Default constructor
     * \param in the inertia properties of the links
     * \param tr the container of all the spatial motion transforms of
     *     the robot Acrobot, which will be used by this instance
     *     to compute the dynamics.
     */
    ForwardDynamics(InertiaProperties& in, MotionTransforms& tr);
    /** \name Forward dynamics
     * The Articulated-Body-Algorithm to compute the joint accelerations
     */ ///@{
    /**
     * \param qdd the joint accelerations vector (output parameter).
     * \param q the joint status vector
     * \param qd the joint velocities vector
     * \param tau the joint forces (torque or force)
     * \param fext the external forces, optional. Each force must be
     *              expressed in the reference frame of the link it is
     *              exerted on.
     */
    void fd(
        JointState& qdd, // output parameter
        const JointState& q, const JointState& qd, const JointState& tau, const ExtForces& fext = zeroExtForces);
    void fd(
        JointState& qdd, // output parameter
        const JointState& qd, const JointState& tau, const ExtForces& fext = zeroExtForces);
    ///@}

    /** Updates all the kinematics transforms used by this instance. */
    void setJointStatus(const JointState& q) const;

private:
    InertiaProperties* inertiaProps;
    MotionTransforms* motionTransforms;

    Matrix66 vcross; // support variable
    Matrix66 Ia_r;   // support variable, articulated inertia in the case of a revolute joint

    // Link 'link1' :
    Matrix66 link1_AI;
    Velocity link1_a;
    Velocity link1_v;
    Velocity link1_c;
    Force    link1_p;

    Column6 link1_U;
    Scalar link1_D;
    Scalar link1_u;
    // Link 'link2' :
    Matrix66 link2_AI;
    Velocity link2_a;
    Velocity link2_v;
    Velocity link2_c;
    Force    link2_p;

    Column6 link2_U;
    Scalar link2_D;
    Scalar link2_u;
private:
    static const ExtForces zeroExtForces;
};

inline void ForwardDynamics::setJointStatus(const JointState& q) const {
    (motionTransforms-> fr_link1_X_fr_base0)(q);
    (motionTransforms-> fr_link2_X_fr_link1)(q);
}

inline void ForwardDynamics::fd(
    JointState& qdd,
    const JointState& q,
    const JointState& qd,
    const JointState& tau,
    const ExtForces& fext/* = zeroExtForces */)
{
    setJointStatus(q);
    fd(qdd, qd, tau, fext);
}

}
}
}

#endif
