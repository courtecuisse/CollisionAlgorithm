#include <sofa/collisionAlgorithm/BaseGeometry.h>
#include <sofa/collisionAlgorithm/BroadPhase.h>

namespace sofa {

namespace collisionAlgorithm {

BroadPhase * BaseGeometry::getBroadPhase() const {
    if (m_broadPhase) m_broadPhase->update(this->getContext()->getTime());
    return m_broadPhase;
}

}

}

