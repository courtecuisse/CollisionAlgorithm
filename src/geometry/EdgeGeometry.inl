#pragma once

#include <geometry/EdgeGeometry.h>
#include <element/EdgeElement.h>
#include <GL/gl.h>

namespace collisionAlgorithm {

void EdgeGeometry::init() {
    m_elements.clear();

    for (unsigned i=0;i<p_topology->getNbEdges();i++) {
        m_elements.push_back(std::make_shared<EdgeElement>(this,i));
    }
}

void EdgeGeometry::prepareDetection() {}

}
