#include <BaseGeometry.h>
#include <element/BaseElement.h>

namespace collisionAlgorithm {

ConstraintProximity::ConstraintProximity(ConstraintElement * elmt)
: m_element(elmt) {
    m_state = m_element->m_geometry->getState();
}

}