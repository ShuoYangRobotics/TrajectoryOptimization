#ifndef IIT_ROBOT_ACROBOT_DECLARATIONS_H_
#define IIT_ROBOT_ACROBOT_DECLARATIONS_H_

#include "rbd_types.h"

namespace iit {
namespace Acrobot {

static constexpr int JointSpaceDimension = 2;
static constexpr int jointsCount = 2;
/** The total number of rigid bodies of this robot, including the base */
static constexpr int linksCount  = 3;

typedef Matrix<2, 1> Column2d;
typedef Column2d JointState;

enum JointIdentifiers {
    JA = 0
    , JB
};

enum LinkIdentifiers {
    BASE0 = 0
    , LINK1
    , LINK2
};

static const JointIdentifiers orderedJointIDs[jointsCount] =
    {JA,JB};

static const LinkIdentifiers orderedLinkIDs[linksCount] =
    {BASE0,LINK1,LINK2};

}
}
#endif
