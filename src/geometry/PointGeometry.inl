#pragma once

#include <geometry/PointGeometry.h>
#include <element/PointElement.h>

namespace sofa {

namespace collisionAlgorithm {

void PointGeometry::initialize() {
    m_elements.clear();
    if (d_topology == NULL) return;

    for (unsigned i=0;i<(unsigned) d_topology->getNbPoints();i++) {
        m_elements.push_back(std::make_shared<PointElement>(this,i));
    }
}

void PointGeometry::prepareDetection() {
    if (m_elements.size() != (unsigned) d_topology->getNbPoints()) init();
}

}

}
