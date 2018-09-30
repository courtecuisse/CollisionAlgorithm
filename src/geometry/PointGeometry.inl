#pragma once

#include <geometry/PointGeometry.h>
#include <element/PointElement.h>
#include <qopengl.h>

namespace sofa {

namespace collisionAlgorithm {

void PointGeometry::init() {
    m_elements.clear();

    for (unsigned i=0;i<(unsigned) m_topology->getNbPoints();i++) {
        m_elements.push_back(std::make_shared<PointElement>(this,i));
    }
}

void PointGeometry::prepareDetection() {
    if (m_elements.size() != (unsigned) m_topology->getNbPoints()) init();
}

}

}
