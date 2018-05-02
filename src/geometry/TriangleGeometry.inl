#pragma once

#include <geometry/TriangleGeometry.h>
#include <element/TriangleElement.h>
#include <GL/gl.h>

namespace collisionAlgorithm {

void TriangleGeometry::prepareDetection() {
    const ReadAccessor<Vector3> & pos = p_topology->p_state->read(VecCoordId::position());

    m_pointNormal.resize(p_topology->getNbPoints());

    m_triangle_info.resize(p_topology->getTriangles().size());

    for (unsigned t=0;t<this->p_topology->getNbTriangles();t++) {
        TriangleInfo & tinfo = m_triangle_info[t];

        const Topology::Triangle tri = this->p_topology->getTriangle(t);

        //Compute Bezier Positions
        Vector3 p0 = pos[tri[0]];
        Vector3 p1 = pos[tri[1]];
        Vector3 p2 = pos[tri[2]];

        tinfo.v0 = p1 - p0;
        tinfo.v1 = p2 - p0;

        tinfo.d00 = dot(tinfo.v0,tinfo.v0);
        tinfo.d01 = dot(tinfo.v0,tinfo.v1);
        tinfo.d11 = dot(tinfo.v1,tinfo.v1);

        tinfo.invDenom = 1.0 / (tinfo.d00 * tinfo.d11 - tinfo.d01 * tinfo.d01);

        tinfo.ax1 = tinfo.v0;
        tinfo.tn = tinfo.v0.cross(tinfo.v1);
        tinfo.ax2 = tinfo.v0.cross(tinfo.tn);

        tinfo.ax1.normalize();
        tinfo.tn.normalize();
        tinfo.ax2.normalize();
    }

    m_pointNormal.resize(pos.size());
    for (unsigned p=0;p<pos.size();p++) {
        const Topology::TrianglesAroundVertex & tav = this->p_topology->getTrianglesAroundVertex(p);
        m_pointNormal[p] = Vector3(0,0,0);
        for (unsigned t=0;t<tav.size();t++) {
            m_pointNormal[p] += this->m_triangle_info[tav[t]].tn;
        }
        m_pointNormal[p].normalize();
    }
}

void TriangleGeometry::init() {
    m_elements.clear();

    for (unsigned i=0;i<p_topology->getNbTriangles();i++) {
        m_elements.push_back(std::make_shared<TriangleElement>(this,i));
    }
}


//void TriangleGeometry::drawTriangle(const core::visual::VisualParams * vparams,const Vector3 & A,const Vector3 & B, const Vector3 & C) {
//    double delta = 0.05;
//    glColor4f(d_color.getValue()[0],d_color.getValue()[1]-delta,d_color.getValue()[2],d_color.getValue()[3]);helper::gl::glVertexT(A);
//    glColor4f(d_color.getValue()[0],d_color.getValue()[1]-2*delta,d_color.getValue()[2],d_color.getValue()[3]);helper::gl::glVertexT(B); // A<->B

//    if (vparams->displayFlags().getShowWireFrame()) {
//        glColor4f(d_color.getValue()[0],d_color.getValue()[1]-2*delta,d_color.getValue()[2],d_color.getValue()[3]);helper::gl::glVertexT(B);
//    } //A<->B

//    glColor4f(d_color.getValue()[0],d_color.getValue()[1]-0.5*delta,d_color.getValue()[2],d_color.getValue()[3]);helper::gl::glVertexT(C);

//    if (vparams->displayFlags().getShowWireFrame()) {
//        glColor4f(d_color.getValue()[0],d_color.getValue()[1]-0.5*delta,d_color.getValue()[2],d_color.getValue()[3]);helper::gl::glVertexT(C);
//    } // C<->A
//    if (vparams->displayFlags().getShowWireFrame()) {
//        glColor4f(d_color.getValue()[0],d_color.getValue()[1]-delta,d_color.getValue()[2],d_color.getValue()[3]);helper::gl::glVertexT(A);
//    }// C<->A
//}


//void TriangleGeometry::draw(const core::visual::VisualParams * vparams) {

//    if (! vparams->displayFlags().getShowCollisionModels()) return;
//    if (d_color.getValue()[3] == 0.0) return;

//    helper::ReadAccessor<Data <VecCoord> > x = *this->getMstate()->read(core::VecCoordId::position());

//    glDisable(GL_LIGHTING);

//    if (vparams->displayFlags().getShowWireFrame()) glBegin(GL_LINES);
//    else {
////        glEnable(GL_CULL_FACE);
//        glBegin(GL_TRIANGLES);
//    }

//    for(int t=0;t<this->p_topology->getNbTriangles();t++) {
//        const sofa::core::topology::BaseMeshTopology::Triangle tri = this->p_topology->geTriangle(t);

//        //Compute Bezier Positions
//        Vector3 p0 = x[tri[0]];
//        Vector3 p1 = x[tri[1]];
//        Vector3 p2 = x[tri[2]];

//        drawTriangle(vparams,p0,p1,p2);
//    }

//    glEnd();
//}

void TriangleGeometry::draw(const VisualParams *vparams) {
    if (! vparams->displayFlags().getShowCollisionModels()) return;

    const ReadAccessor<Vector3> & pos = p_topology->p_state->read(VecCoordId::position());

    glDisable(GL_LIGHTING);
    glColor4f(1,0,1,1);
    glEnable(GL_CULL_FACE);
    glBegin(GL_TRIANGLES);
    for (unsigned i=0;i<m_elements.size();i++) {
        std::shared_ptr<TriangleElement> elmt = std::dynamic_pointer_cast<TriangleElement>(m_elements[i]);
        glVertex3dv(pos[elmt->m_pid[0]].data());
        glVertex3dv(pos[elmt->m_pid[1]].data());
        glVertex3dv(pos[elmt->m_pid[2]].data());
    }
    glEnd();
}

}
