#include "inertia_properties.h"

using namespace std;
using namespace iit::rbd;

iit::Acrobot::dyn::InertiaProperties::InertiaProperties()
{
    com_link1 = Vector3(comx_link1,0.0,0.0);
    tensor_link1.fill(
        m_link1,
        com_link1,
        Utils::buildInertiaTensor<Scalar>(ix_link1,iy_link1,iz_link1,0.0,0.0,0.0) );

    com_link2 = Vector3(comx_link2,0.0,0.0);
    tensor_link2.fill(
        m_link2,
        com_link2,
        Utils::buildInertiaTensor<Scalar>(ix_link2,iy_link2,iz_link2,0.0,0.0,0.0) );

}


void iit::Acrobot::dyn::InertiaProperties::updateParameters(const RuntimeInertiaParams& fresh)
{
    this-> params = fresh;
}
