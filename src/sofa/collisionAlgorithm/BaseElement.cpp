#include <sofa/collisionAlgorithm/BaseGeometry.h>

namespace sofa {

namespace collisionAlgorithm {

ConstraintProximity::ConstraintProximity(const ConstraintElement * elmt)
{
    m_state = elmt->geometry()->getState();
}

}

}
