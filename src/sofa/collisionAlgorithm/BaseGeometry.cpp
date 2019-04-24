#include <sofa/collisionAlgorithm/BaseGeometry.h>
#include <sofa/collisionAlgorithm/BroadPhase.h>

namespace sofa {

namespace collisionAlgorithm {

const BroadPhase * BaseGeometry::getBroadPhase() {
    updateTime();
    if (m_broadPhase) m_broadPhase->updateTime(m_update_time);
    return m_broadPhase;
}

}

}

