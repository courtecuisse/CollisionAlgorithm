#pragma once

#include <geometry/EdgeGeometry.h>
#include <element/EdgeElement.h>
#include <qopengl.h>

namespace collisionAlgorithm {

void EdgeGeometry::init() {
    m_elements.clear();

    for (unsigned i=0;i<p_topology->getNbEdges();i++) {
        m_elements.push_back(std::make_shared<EdgeElement>(this,i));
    }
}

void EdgeGeometry::prepareDetection() {
    if (m_elements.size() != p_topology->getNbEdges()) init();
}

}
