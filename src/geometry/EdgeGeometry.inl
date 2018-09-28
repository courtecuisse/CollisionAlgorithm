#pragma once

#include <geometry/EdgeGeometry.h>
#include <element/EdgeElement.h>
#include <qopengl.h>

namespace sofa {

namespace collisionAlgorithm {

void EdgeGeometry::init() {
    m_elements.clear();

    for (unsigned i=0;i<m_topology->getNbEdges();i++) {
        m_elements.push_back(std::make_shared<EdgeElement>(this,i));
    }
}

void EdgeGeometry::prepareDetection() {
    if (m_elements.size() != m_topology->getNbEdges()) init();
}

}

}
