#pragma once

#include <geometry/EdgeGeometry.h>
#include <element/EdgeElement.h>

namespace sofa {

namespace collisionAlgorithm {

void EdgeGeometry::initialize() {
    m_elements.clear();

    for (unsigned i=0;i<d_topology->getNbEdges();i++) {
        m_elements.push_back(std::make_shared<EdgeElement>(this,i));
    }
}

void EdgeGeometry::prepareDetection() {
    if (m_elements.size() != d_topology->getNbEdges()) init();
}

}

}
