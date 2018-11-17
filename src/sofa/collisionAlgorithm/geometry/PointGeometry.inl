#pragma once

#include <sofa/collisionAlgorithm/geometry/PointGeometry.h>
#include <sofa/collisionAlgorithm/element/PointElement.h>
#include <sofa/collisionAlgorithm/proximity/PointProximity.h>

namespace sofa {

namespace collisionAlgorithm {

void PointGeometry::init() {
    m_elements.clear();

    for (unsigned i=0;i<(unsigned) l_state->getSize();i++) {
        m_elements.push_back(PointElement::createElement(this,i));
    }
}

void PointGeometry::prepareDetection() {
    if (m_elements.size() != (unsigned) l_state->getSize()) init();
}

ConstraintProximity::SPtr PointGeometry::createProximity(const PointElement * elmt) {
    return std::shared_ptr<PointProximity>(new PointProximity(elmt));
}

}

}
