#ifndef IIT_ROBOT_ACROBOT_INERTIA_PROPERTIES_H_
#define IIT_ROBOT_ACROBOT_INERTIA_PROPERTIES_H_

#include <iit/rbd/rbd.h>
#include <iit/rbd/InertiaMatrix.h>
#include <iit/rbd/utils.h>

#include "declarations.h"
#include "model_constants.h"
#include "dynamics_parameters.h"

namespace iit {
namespace Acrobot {
/**
 * This namespace encloses classes and functions related to the Dynamics
 * of the robot Acrobot.
 */
namespace dyn {

class InertiaProperties {
    public:
        InertiaProperties();
        ~InertiaProperties();
        const InertiaMatrix& getTensor_link1() const;
        const InertiaMatrix& getTensor_link2() const;
        Scalar getMass_link1() const;
        Scalar getMass_link2() const;
        const Vector3& getCOM_link1() const;
        const Vector3& getCOM_link2() const;
        Scalar getTotalMass() const;


        /*!
         * Fresh values for the runtime parameters of the robot Acrobot,
         * causing the update of the inertia properties modeled by this
         * instance.
         */
        void updateParameters(const RuntimeInertiaParams&);

    private:
        RuntimeInertiaParams params;

        InertiaMatrix tensor_link1;
        InertiaMatrix tensor_link2;
        Vector3 com_link1;
        Vector3 com_link2;
};


inline InertiaProperties::~InertiaProperties() {}

inline const InertiaMatrix& InertiaProperties::getTensor_link1() const {
    return this->tensor_link1;
}
inline const InertiaMatrix& InertiaProperties::getTensor_link2() const {
    return this->tensor_link2;
}
inline Scalar InertiaProperties::getMass_link1() const {
    return this->tensor_link1.getMass();
}
inline Scalar InertiaProperties::getMass_link2() const {
    return this->tensor_link2.getMass();
}
inline const Vector3& InertiaProperties::getCOM_link1() const {
    return this->com_link1;
}
inline const Vector3& InertiaProperties::getCOM_link2() const {
    return this->com_link2;
}

inline Scalar InertiaProperties::getTotalMass() const {
    return m_link1 + m_link2;
}

}
}
}

#endif
