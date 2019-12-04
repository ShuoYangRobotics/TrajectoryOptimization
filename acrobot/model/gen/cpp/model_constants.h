#ifndef IIT_ROBOT_ACROBOT_MODEL_CONSTANTS_H_
#define IIT_ROBOT_ACROBOT_MODEL_CONSTANTS_H_

#include "rbd_types.h"

/**
 * \file
 * This file contains the definitions of all the non-zero numerical
 * constants of the robot model (i.e. the numbers appearing in the
 * .kindsl file).
 *
 * Varying these values (and recompiling) is a quick & dirty
 * way to vary the kinematics/dynamics model. For a much more
 * flexible way of exploring variations of the model, consider
 * using the parametrization feature of RobCoGen (see the wiki).
 *
 * Beware of inconsistencies when changing any of the inertia
 * properties.
 */

namespace iit {
namespace Acrobot {

// Do not use 'constexpr' to allow for non-literal scalar types

const Scalar tx_jB = -1.0;
const Scalar tx_ee = -1.0;
const Scalar m_base0 = 1.0;
const Scalar m_link1 = 1.0;
const Scalar comx_link1 = -0.5;
const Scalar ix_link1 = 0.0024999999441206455;
const Scalar iy_link1 = 0.3345800042152405;
const Scalar iz_link1 = 0.3345800042152405;
const Scalar m_link2 = 1.0;
const Scalar comx_link2 = -0.5;
const Scalar ix_link2 = 0.0024999999441206455;
const Scalar iy_link2 = 0.3345800042152405;
const Scalar iz_link2 = 0.3345800042152405;

}
}
#endif
