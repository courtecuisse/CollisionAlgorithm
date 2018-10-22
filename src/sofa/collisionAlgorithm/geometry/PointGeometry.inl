#pragma once

#include <sofa/collisionAlgorithm/geometry/PointGeometry.h>
#include <sofa/collisionAlgorithm/element/PointElement.h>
#include <sofa/collisionAlgorithm/proximity/PointProximity.h>

namespace sofa {

namespace collisionAlgorithm {

void PointGeometry::init() {
    m_elements.clear();
    if (d_topology == NULL) return;

    for (unsigned i=0;i<(unsigned) d_topology->getNbPoints();i++) {
        m_elements.push_back(PointElement::createElement(this,i));
    }
}

void PointGeometry::prepareDetection() {
    if (m_elements.size() != (unsigned) d_topology->getNbPoints()) init();
}

ConstraintProximity::SPtr PointGeometry::createProximity(const PointElement * elmt) {
    return std::shared_ptr<PointProximity>(new PointProximity(elmt));
}

}

}
