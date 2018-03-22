#pragma once

#include <geometry/EdgeGeometry.h>
#include <GL/gl.h>

namespace graFE {

/**************************************************************************/
/******************************PROXIMITY***********************************/
/**************************************************************************/

EdgeProximity::EdgeProximity(EdgeElement * geo,double f1,double f2) {
    m_elmt = geo;
    m_fact[0] = f1;
    m_fact[1] = f2;
}

Vector3 EdgeProximity::getPosition() const {
    const std::vector<Vector3> & pos = m_elmt->m_geo->getPos();
    return pos[m_elmt->m_pid[0]] * m_fact[0] + pos[m_elmt->m_pid[1]] * m_fact[1];
}

Vector3 EdgeProximity::getFreePosition() const {
    const std::vector<Vector3> & pos = m_elmt->m_geo->getFreePos();
    return pos[m_elmt->m_pid[0]] * m_fact[0] + pos[m_elmt->m_pid[1]] * m_fact[1];
}

Vector3 EdgeProximity::getNormal() const {
    return Vector3(1,0,0);
}

ConstraintElement * EdgeProximity::getElement() {
    return m_elmt;
}

/**************************************************************************/
/******************************ELEMENT*************************************/
/**************************************************************************/

EdgeElement::EdgeElement(EdgeGeometry * geo,unsigned eid) {
    m_geo = geo;
    m_eid = eid;

    const std::vector<TEdge> & edges = m_geo->getTopology()->getEdges();

    m_pid[0] = edges[eid][0];
    m_pid[1] = edges[eid][1];
}

ConstraintProximityPtr EdgeElement::getControlPoint(const int cid) {
    if (cid == 0) return std::make_shared<EdgeProximity>(this,1,0);
    else if (cid == 1) return std::make_shared<EdgeProximity>(this,0,1);
    return NULL;
}

unsigned EdgeElement::getNbControlPoints() {
    return 2;
}

//this function project the point P on the element and return the corresponding proximity
ConstraintProximityPtr EdgeElement::project(Vector3 P) {
    double fact_u,fact_v;

    const std::vector<Vector3> & pos = m_geo->getPos();

    Vector3 P1 = pos[m_pid[0]];
    Vector3 P2 = pos[m_pid[1]];

    Vector3 v = P2-P1;
    fact_v = (P - P1).dot(v) / v.dot(v);

    if (fact_v<0.0) fact_v = 0.0;
    else if (fact_v>1.0) fact_v = 1.0;

    fact_u = 1.0-fact_v;

    return std::make_shared<EdgeProximity>(this,fact_u,fact_v);
}

void EdgeElement::draw(const std::vector<Vector3> & X) {
    glBegin(GL_LINES);
        glVertex3dv(X[m_pid[0]].data());
        glVertex3dv(X[m_pid[1]].data());
    glEnd();
}

/**************************************************************************/
/******************************GEOMETRY************************************/
/**************************************************************************/

void EdgeGeometry::createElements() {
    for (unsigned i=0;i<getTopology()->getEdges().size();i++) {
        m_elements.push_back(std::make_shared<EdgeElement>(this,i));
    }
}

}
