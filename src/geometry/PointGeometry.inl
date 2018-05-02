#pragma once

#include <geometry/PointGeometry.h>
#include <element/PointElement.h>
#include <GL/gl.h>

namespace collisionAlgorithm {

void PointGeometry::init() {
    m_elements.clear();

    for (unsigned i=0;i<p_topology->getNbPoints();i++) {
        m_elements.push_back(std::make_shared<PointElement>(this,i));
    }
}

void PointGeometry::prepareDetection() {}

void PointGeometry::draw(const VisualParams *vparams) {
    if (! vparams->displayFlags().getShowCollisionModels()) return;

    const ReadAccessor<Vector3> & pos = p_topology->p_state->read(VecCoordId::position());

    glDisable(GL_LIGHTING);
    glColor4f(1,0,1,1);
    glBegin(GL_POINTS);
    glPointSize(20);
    for (unsigned i=0;i<m_elements.size();i++) {
        std::shared_ptr<PointElement> elmt = std::dynamic_pointer_cast<PointElement>(m_elements[i]);
        glVertex3dv(pos[elmt->m_pid].data());
    }
    glEnd();
}

}
