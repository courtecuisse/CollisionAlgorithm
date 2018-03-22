#pragma once

#include <geometry/PointGeometry.h>
#include <GL/gl.h>

namespace collisionAlgorithm {

/**************************************************************************/
/******************************PROXIMITY***********************************/
/**************************************************************************/

PointProximity::PointProximity(PointElement * geo) {
    m_elmt = geo;
}

Vector3 PointProximity::getPosition() const {
    return m_elmt->m_geo->getPos()[m_elmt->m_pid];
}

Vector3 PointProximity::getFreePosition() const {
    return m_elmt->m_geo->getFreePos()[m_elmt->m_pid];
}

Vector3 PointProximity::getNormal() const {
    return Vector3(1,0,0);
}

ConstraintElement * PointProximity::getElement() {
    return m_elmt;
}

std::map<unsigned,Vector3> PointProximity::getContribution(const Vector3 & N) {
    std::map<unsigned,Vector3> res;

    res[m_elmt->m_pid] = N;

    return res;
}

/**************************************************************************/
/******************************ELEMENT*************************************/
/**************************************************************************/

PointElement::PointElement(PointGeometry * geo,unsigned pid) {
    m_pid = pid;
    m_geo = geo;
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

void PointGeometry::createElements() {
    for (unsigned i=0;i<getPos().size();i++) {
        m_elements.push_back(std::make_shared<PointElement>(this,i));
    }
}


}
