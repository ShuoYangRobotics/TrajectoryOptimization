#ifndef ACROBOT_TRANSFORMS_H_
#define ACROBOT_TRANSFORMS_H_

#include <iit/rbd/TransformsBase.h>
#include "declarations.h"
#include "model_constants.h"
#include "kinematics_parameters.h"

namespace iit {
namespace Acrobot {

struct Parameters
{
    struct AngleFuncValues {
        AngleFuncValues() {
            update();
        }

        void update()
        {
        }
    };

    Params_lengths lengths;
    Params_angles angles;
    AngleFuncValues trig = AngleFuncValues();
};

// The type of the "vector" with the status of the variables
typedef JointState state_t;

template<class M>
using TransformMotion = iit::rbd::SpatialTransformBase<state_t, M>;

template<class M>
using TransformForce = iit::rbd::SpatialTransformBase<state_t, M>;

template<class M>
using TransformHomogeneous = iit::rbd::HomogeneousTransformBase<state_t, M>;

/**
 * The class for the 6-by-6 coordinates transformation matrices for
 * spatial motion vectors.
 */
class MotionTransforms
{
public:
    class Dummy {};
    typedef TransformMotion<Dummy>::MatrixType MatrixType;

    struct Type_fr_base0_X_ee : public TransformMotion<Type_fr_base0_X_ee>
    {
        Type_fr_base0_X_ee();
        const Type_fr_base0_X_ee& update(const state_t&);
    };
    
    struct Type_fr_jA_X_ee : public TransformMotion<Type_fr_jA_X_ee>
    {
        Type_fr_jA_X_ee();
        const Type_fr_jA_X_ee& update(const state_t&);
    };
    
    struct Type_fr_jB_X_ee : public TransformMotion<Type_fr_jB_X_ee>
    {
        Type_fr_jB_X_ee();
        const Type_fr_jB_X_ee& update(const state_t&);
    };
    
    struct Type_fr_base0_X_fr_jA : public TransformMotion<Type_fr_base0_X_fr_jA>
    {
        Type_fr_base0_X_fr_jA();
        const Type_fr_base0_X_fr_jA& update(const state_t&);
    };
    
    struct Type_fr_base0_X_fr_jB : public TransformMotion<Type_fr_base0_X_fr_jB>
    {
        Type_fr_base0_X_fr_jB();
        const Type_fr_base0_X_fr_jB& update(const state_t&);
    };
    
    struct Type_fr_link1_X_fr_base0 : public TransformMotion<Type_fr_link1_X_fr_base0>
    {
        Type_fr_link1_X_fr_base0();
        const Type_fr_link1_X_fr_base0& update(const state_t&);
    };
    
    struct Type_fr_base0_X_fr_link1 : public TransformMotion<Type_fr_base0_X_fr_link1>
    {
        Type_fr_base0_X_fr_link1();
        const Type_fr_base0_X_fr_link1& update(const state_t&);
    };
    
    struct Type_fr_link2_X_fr_link1 : public TransformMotion<Type_fr_link2_X_fr_link1>
    {
        Type_fr_link2_X_fr_link1();
        const Type_fr_link2_X_fr_link1& update(const state_t&);
    };
    
    struct Type_fr_link1_X_fr_link2 : public TransformMotion<Type_fr_link1_X_fr_link2>
    {
        Type_fr_link1_X_fr_link2();
        const Type_fr_link1_X_fr_link2& update(const state_t&);
    };
    
public:
    MotionTransforms();
    void updateParams(const Params_lengths&, const Params_angles&);

    Type_fr_base0_X_ee fr_base0_X_ee;
    Type_fr_jA_X_ee fr_jA_X_ee;
    Type_fr_jB_X_ee fr_jB_X_ee;
    Type_fr_base0_X_fr_jA fr_base0_X_fr_jA;
    Type_fr_base0_X_fr_jB fr_base0_X_fr_jB;
    Type_fr_link1_X_fr_base0 fr_link1_X_fr_base0;
    Type_fr_base0_X_fr_link1 fr_base0_X_fr_link1;
    Type_fr_link2_X_fr_link1 fr_link2_X_fr_link1;
    Type_fr_link1_X_fr_link2 fr_link1_X_fr_link2;

protected:
    Parameters params;

}; //class 'MotionTransforms'

/**
 * The class for the 6-by-6 coordinates transformation matrices for
 * spatial force vectors.
 */
class ForceTransforms
{
public:
    class Dummy {};
    typedef TransformForce<Dummy>::MatrixType MatrixType;

    struct Type_fr_base0_X_ee : public TransformForce<Type_fr_base0_X_ee>
    {
        Type_fr_base0_X_ee();
        const Type_fr_base0_X_ee& update(const state_t&);
    };
    
    struct Type_fr_jA_X_ee : public TransformForce<Type_fr_jA_X_ee>
    {
        Type_fr_jA_X_ee();
        const Type_fr_jA_X_ee& update(const state_t&);
    };
    
    struct Type_fr_jB_X_ee : public TransformForce<Type_fr_jB_X_ee>
    {
        Type_fr_jB_X_ee();
        const Type_fr_jB_X_ee& update(const state_t&);
    };
    
    struct Type_fr_base0_X_fr_jA : public TransformForce<Type_fr_base0_X_fr_jA>
    {
        Type_fr_base0_X_fr_jA();
        const Type_fr_base0_X_fr_jA& update(const state_t&);
    };
    
    struct Type_fr_base0_X_fr_jB : public TransformForce<Type_fr_base0_X_fr_jB>
    {
        Type_fr_base0_X_fr_jB();
        const Type_fr_base0_X_fr_jB& update(const state_t&);
    };
    
    struct Type_fr_link1_X_fr_base0 : public TransformForce<Type_fr_link1_X_fr_base0>
    {
        Type_fr_link1_X_fr_base0();
        const Type_fr_link1_X_fr_base0& update(const state_t&);
    };
    
    struct Type_fr_base0_X_fr_link1 : public TransformForce<Type_fr_base0_X_fr_link1>
    {
        Type_fr_base0_X_fr_link1();
        const Type_fr_base0_X_fr_link1& update(const state_t&);
    };
    
    struct Type_fr_link2_X_fr_link1 : public TransformForce<Type_fr_link2_X_fr_link1>
    {
        Type_fr_link2_X_fr_link1();
        const Type_fr_link2_X_fr_link1& update(const state_t&);
    };
    
    struct Type_fr_link1_X_fr_link2 : public TransformForce<Type_fr_link1_X_fr_link2>
    {
        Type_fr_link1_X_fr_link2();
        const Type_fr_link1_X_fr_link2& update(const state_t&);
    };
    
public:
    ForceTransforms();
    void updateParams(const Params_lengths&, const Params_angles&);

    Type_fr_base0_X_ee fr_base0_X_ee;
    Type_fr_jA_X_ee fr_jA_X_ee;
    Type_fr_jB_X_ee fr_jB_X_ee;
    Type_fr_base0_X_fr_jA fr_base0_X_fr_jA;
    Type_fr_base0_X_fr_jB fr_base0_X_fr_jB;
    Type_fr_link1_X_fr_base0 fr_link1_X_fr_base0;
    Type_fr_base0_X_fr_link1 fr_base0_X_fr_link1;
    Type_fr_link2_X_fr_link1 fr_link2_X_fr_link1;
    Type_fr_link1_X_fr_link2 fr_link1_X_fr_link2;

protected:
    Parameters params;

}; //class 'ForceTransforms'

/**
 * The class with the homogeneous (4x4) coordinates transformation
 * matrices.
 */
class HomogeneousTransforms
{
public:
    class Dummy {};
    typedef TransformHomogeneous<Dummy>::MatrixType MatrixType;

    struct Type_fr_base0_X_ee : public TransformHomogeneous<Type_fr_base0_X_ee>
    {
        Type_fr_base0_X_ee();
        const Type_fr_base0_X_ee& update(const state_t&);
    };
    
    struct Type_fr_jA_X_ee : public TransformHomogeneous<Type_fr_jA_X_ee>
    {
        Type_fr_jA_X_ee();
        const Type_fr_jA_X_ee& update(const state_t&);
    };
    
    struct Type_fr_jB_X_ee : public TransformHomogeneous<Type_fr_jB_X_ee>
    {
        Type_fr_jB_X_ee();
        const Type_fr_jB_X_ee& update(const state_t&);
    };
    
    struct Type_fr_base0_X_fr_jA : public TransformHomogeneous<Type_fr_base0_X_fr_jA>
    {
        Type_fr_base0_X_fr_jA();
        const Type_fr_base0_X_fr_jA& update(const state_t&);
    };
    
    struct Type_fr_base0_X_fr_jB : public TransformHomogeneous<Type_fr_base0_X_fr_jB>
    {
        Type_fr_base0_X_fr_jB();
        const Type_fr_base0_X_fr_jB& update(const state_t&);
    };
    
    struct Type_fr_link1_X_fr_base0 : public TransformHomogeneous<Type_fr_link1_X_fr_base0>
    {
        Type_fr_link1_X_fr_base0();
        const Type_fr_link1_X_fr_base0& update(const state_t&);
    };
    
    struct Type_fr_base0_X_fr_link1 : public TransformHomogeneous<Type_fr_base0_X_fr_link1>
    {
        Type_fr_base0_X_fr_link1();
        const Type_fr_base0_X_fr_link1& update(const state_t&);
    };
    
    struct Type_fr_link2_X_fr_link1 : public TransformHomogeneous<Type_fr_link2_X_fr_link1>
    {
        Type_fr_link2_X_fr_link1();
        const Type_fr_link2_X_fr_link1& update(const state_t&);
    };
    
    struct Type_fr_link1_X_fr_link2 : public TransformHomogeneous<Type_fr_link1_X_fr_link2>
    {
        Type_fr_link1_X_fr_link2();
        const Type_fr_link1_X_fr_link2& update(const state_t&);
    };
    
public:
    HomogeneousTransforms();
    void updateParams(const Params_lengths&, const Params_angles&);

    Type_fr_base0_X_ee fr_base0_X_ee;
    Type_fr_jA_X_ee fr_jA_X_ee;
    Type_fr_jB_X_ee fr_jB_X_ee;
    Type_fr_base0_X_fr_jA fr_base0_X_fr_jA;
    Type_fr_base0_X_fr_jB fr_base0_X_fr_jB;
    Type_fr_link1_X_fr_base0 fr_link1_X_fr_base0;
    Type_fr_base0_X_fr_link1 fr_base0_X_fr_link1;
    Type_fr_link2_X_fr_link1 fr_link2_X_fr_link1;
    Type_fr_link1_X_fr_link2 fr_link1_X_fr_link2;

protected:
    Parameters params;

}; //class 'HomogeneousTransforms'

}
}

#endif
