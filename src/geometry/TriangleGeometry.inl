#pragma once

#include <geometry/TriangleGeometry.h>
#include <GL/gl.h>

namespace collisionAlgorithm {

/**************************************************************************/
/******************************PROXIMITY***********************************/
/**************************************************************************/

TriangleProximity::TriangleProximity(TriangleElement * elmt,double f1,double f2,double f3) : ConstraintProximity (elmt) {
    m_fact[0] = f1;
    m_fact[1] = f2;
    m_fact[2] = f3;
}

Vector3 TriangleProximity::getPosition(TVecId v) const {
    const std::vector<Vector3> & pos = element()->geometry()->getPos(v);
    return pos[element()->m_pid[0]] * m_fact[0] +
           pos[element()->m_pid[1]] * m_fact[1] +
           pos[element()->m_pid[2]] * m_fact[2];
}

Vector3 TriangleProximity::getNormal() const {
    const std::vector<Vector3> & pos = element()->geometry()->m_pointNormal;
    return pos[element()->m_pid[0]] * m_fact[0] +
           pos[element()->m_pid[1]] * m_fact[1] +
           pos[element()->m_pid[2]] * m_fact[2];
}

std::map<unsigned,Vector3> TriangleProximity::getContribution(const Vector3 & N) {
    std::map<unsigned,Vector3> res;

    res[element()->m_pid[0]] = N * 1.0/3.0;
    res[element()->m_pid[1]] = N * 1.0/3.0;
    res[element()->m_pid[2]] = N * 1.0/3.0;

    return res;
}

/**************************************************************************/
/******************************ELEMENT*************************************/
/**************************************************************************/

TriangleElement::TriangleElement(TriangleGeometry * geo,unsigned eid) : ConstraintElement(geo,3) {
    m_eid = eid;

    const std::vector<TTriangle> & triangles = geometry()->p_topology->getTriangles();

    m_pid[0] = triangles[eid][0];
    m_pid[1] = triangles[eid][1];
    m_pid[2] = triangles[eid][2];
}

ConstraintProximityPtr TriangleElement::getControlPoint(const int cid) {
    if (cid == 0) return std::make_shared<TriangleProximity>(this,1,0,0);
    else if (cid == 1) return std::make_shared<TriangleProximity>(this,0,1,0);
    else if (cid == 2) return std::make_shared<TriangleProximity>(this,0,0,1);
    return std::make_shared<TriangleProximity>(this,1.0/3.0,1.0/3.0,1.0/3.0);
}

//proj_P must be on the plane

void TriangleElement::computeBaryCoords(const Vector3 & proj_P,const TriangleGeometry::TriangleInfo & tinfo, const Vector3 & p0, double & fact_u,double & fact_v, double & fact_w) const {
    Vector3 v2 = proj_P - p0;

    double d20 = dot(v2,tinfo.v0);
    double d21 = dot(v2,tinfo.v1);

    fact_v = (tinfo.d11 * d20 - tinfo.d01 * d21) * tinfo.invDenom;
    fact_w = (tinfo.d00 * d21 - tinfo.d01 * d20) * tinfo.invDenom;
    fact_u = 1.0 - fact_v  - fact_w;
}

//Barycentric coordinates are computed according to
//http://gamedev.stackexchange.com/questions/23743/whats-the-most-efficient-way-to-find-barycentric-coordinates

ConstraintProximityPtr TriangleElement::project(Vector3 P) {
    const std::vector<Vector3> & pos = geometry()->getPos(TVecId::position);

    Vector3 P0 = pos[m_pid[0]];
    Vector3 P1 = pos[m_pid[1]];
    Vector3 P2 = pos[m_pid[2]];

    Vector3 x1x2 = P - P0;

    const TriangleGeometry::TriangleInfo & tinfo = geometry()->m_triangle_info[m_eid];

    //corrdinate on the plane
    double c0 = dot(x1x2,tinfo.ax1);
    double c1 = dot(x1x2,tinfo.ax2);
    Vector3 proj_P = P0 + tinfo.ax1 * c0 + tinfo.ax2 * c1;

    double fact_u,fact_v,fact_w;

    computeBaryCoords(proj_P, tinfo, P0, fact_u,fact_v,fact_w);

    if (fact_u<0) {
        Vector3 v3 = P1 - P2;
        Vector3 v4 = proj_P - P2;
        double alpha = dot(v4,v3) / dot(v3,v3);

        if (alpha<0) alpha = 0;
        else if (alpha>1) alpha = 1;

        fact_u = 0;
        fact_v = alpha;
        fact_w = 1.0 - alpha;
    } else if (fact_v<0) {
        Vector3 v3 = P0 - P2;
        Vector3 v4 = proj_P - P2;
        double alpha = dot(v4,v3) / dot(v3,v3);

        if (alpha<0) alpha = 0;
        else if (alpha>1) alpha = 1;

        fact_u = alpha;
        fact_v = 0;
        fact_w = 1.0 - alpha;
    } else if (fact_w<0) {
        Vector3 v3 = P1 - P0;
        Vector3 v4 = proj_P - P0;
        double alpha = dot(v4,v3) / dot(v3,v3);

        if (alpha<0) alpha = 0;
        else if (alpha>1) alpha = 1;

        fact_u = 1.0 - alpha;
        fact_v = alpha;
        fact_w = 0;
    }

    return std::make_shared<TriangleProximity>(this,fact_u,fact_v,fact_w);
}

void TriangleElement::draw(const std::vector<Vector3> & X) {
    glEnable(GL_CULL_FACE);
    glBegin(GL_TRIANGLES);
        glVertex3dv(X[m_pid[0]].data());
        glVertex3dv(X[m_pid[1]].data());
        glVertex3dv(X[m_pid[2]].data());
    glEnd();
}

//void TriangleGeometry::drawTriangle(const core::visual::VisualParams * vparams,const defaulttype::Vector3 & A,const defaulttype::Vector3 & B, const defaulttype::Vector3 & C) {
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
//        const sofa::core::topology::BaseMeshTopology::Triangle tri = this->p_topology->getTriangle(t);

//        //Compute Bezier Positions
//        defaulttype::Vector3 p0 = x[tri[0]];
//        defaulttype::Vector3 p1 = x[tri[1]];
//        defaulttype::Vector3 p2 = x[tri[2]];

//        drawTriangle(vparams,p0,p1,p2);
//    }

//    glEnd();
//}

/**************************************************************************/
/******************************GEOMETRY************************************/
/**************************************************************************/

void TriangleGeometry::prepareDetection() {
    const std::vector<Vector3> & x = getPos(TVecId::position);

    m_pointNormal.resize(p_topology->getNbPoints());

    m_triangle_info.resize(p_topology->getTriangles().size());

    for (unsigned t=0;t<this->p_topology->getNbTriangles();t++) {
        TriangleInfo & tinfo = m_triangle_info[t];

        const TTriangle tri = this->p_topology->getTriangle(t);

        //Compute Bezier Positions
        Vector3 p0 = x[tri[0]];
        Vector3 p1 = x[tri[1]];
        Vector3 p2 = x[tri[2]];

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

    m_pointNormal.resize(x.size());
    for (unsigned p=0;p<x.size();p++) {
        const Topology::TrianglesAroundVertex & tav = this->p_topology->getTrianglesAroundVertex(p);
        m_pointNormal[p] = Vector3(0,0,0);
        for (unsigned t=0;t<tav.size();t++) {
            m_pointNormal[p] += this->m_triangle_info[tav[t]].tn;
        }
        m_pointNormal[p].normalize();
    }

    if (!m_elements.empty()) return;

    for (unsigned i=0;i<p_topology->getNbTriangles();i++) {
        m_elements.push_back(std::make_shared<TriangleElement>(this,i));
    }
}

}
