#include <BaseGeometry.h>

namespace sofa {

namespace collisionAlgorithm {

ConstraintProximity::ConstraintProximity(const ConstraintElement * elmt)
: m_element(elmt) {
    m_state = m_element->m_geometry->getState();
}

}

}
