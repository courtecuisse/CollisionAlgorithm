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

void EdgeGeometry::draw(const VisualParams *vparams) {
    if (! vparams->displayFlags().getShowCollisionModels()) return;

    const ReadAccessor<Vector3> & pos = p_topology->p_state->read(VecCoordId::position());

    glDisable(GL_LIGHTING);
    glColor4f(1,0,1,1);
    glBegin(GL_LINES);
    for (unsigned i=0;i<m_elements.size();i++) {
        std::shared_ptr<EdgeElement> elmt = std::dynamic_pointer_cast<EdgeElement>(m_elements[i]);
        glVertex3dv(pos[elmt->m_pid[0]].data());
        glVertex3dv(pos[elmt->m_pid[1]].data());
    }
    glEnd();
}

}
