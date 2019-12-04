#ifndef ACROBOT_JACOBIANS_H_
#define ACROBOT_JACOBIANS_H_

#include <iit/rbd/TransformsBase.h>
#include "declarations.h"
#include "kinematics_parameters.h"
#include "model_constants.h"

namespace iit {
namespace Acrobot {

template<int COLS, class M>
class JacobianT : public iit::rbd::JacobianBase<JointState, COLS, M>
{};

/**
 *
 */
class Jacobians
{
    public:
        class Type_fr_base0_J_ee : public JacobianT<2, Type_fr_base0_J_ee>
        {
        public:
            Type_fr_base0_J_ee();
            const Type_fr_base0_J_ee& update(const JointState&);
        protected:
        };
        
    public:
        Jacobians();
        void updateParameters(const Params_lengths& _lengths, const Params_angles& _angles);
    public:
        Type_fr_base0_J_ee fr_base0_J_ee;

    protected:
        Params_lengths lengths;
        Params_angles angles;
};


}
}

#endif
