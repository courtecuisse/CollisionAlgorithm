#pragma once

#include <geometry/PointGeometry.h>
#include <GL/gl.h>

namespace collisionAlgorithm {

/**************************************************************************/
/******************************PROXIMITY***********************************/
/**************************************************************************/

PointProximity::PointProximity(PointElement * elmt) : ConstraintProximity(elmt) {}

Vector3 PointProximity::getPosition(TVecId v) const {
    return element()->geometry()->getPos(TVecId::position)[element()->m_pid];
}

Vector3 PointProximity::getNormal() const {
    return Vector3(1,0,0);
}

std::map<unsigned,Vector3> PointProximity::getContribution(const Vector3 & N) {
    std::map<unsigned,Vector3> res;

    res[element()->m_pid] = N;

    return res;
}

/**************************************************************************/
/******************************ELEMENT*************************************/
/**************************************************************************/

PointElement::PointElement(PointGeometry * geo,unsigned pid) : ConstraintElement(geo) {
    m_pid = pid;
}

ConstraintProximityPtr PointElement::getControlPoint(const int cid) {
    if (cid == 0) return std::make_shared<PointProximity>(this);
    return NULL;
}

unsigned PointElement::getNbControlPoints() {
    return 1;
}

//this function project the point P on the element and return the corresponding proximity
ConstraintProximityPtr PointElement::project(Vector3 /*P*/) {
    return getControlPoint(0);
}

void PointElement::draw(const std::vector<Vector3> & X) {
    glBegin(GL_POINTS);
    glPointSize(20);
    glVertex3dv(X[m_pid].data());
    glEnd();
}


/**************************************************************************/
/******************************GEOMETRY************************************/
/**************************************************************************/

void PointGeometry::prepareDetection() {
    if (! m_elements.empty()) return;

    for (unsigned i=0;i<p_topology->getNbPoints();i++) {
        m_elements.push_back(std::make_shared<PointElement>(this,i));
    }
}


}
