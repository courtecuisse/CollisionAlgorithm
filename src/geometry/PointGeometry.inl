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

PointElement::PointElement(PointGeometry * geo,unsigned pid) : ConstraintElement(geo,1) {
    m_pid = pid;
}

ConstraintProximityPtr PointElement::getControlPoint(const int /*cid*/) {
    return std::make_shared<PointProximity>(this);
}

//this function project the point P on the element and return the corresponding proximity
ConstraintProximityPtr PointElement::project(Vector3 /*P*/) {
    return getControlPoint(0);
}

/**************************************************************************/
/******************************GEOMETRY************************************/
/**************************************************************************/

void PointGeometry::init() {
    m_elements.clear();

    for (unsigned i=0;i<p_topology->getNbPoints();i++) {
        m_elements.push_back(std::make_shared<PointElement>(this,i));
    }
}

void PointGeometry::prepareDetection() {}

void PointGeometry::draw(const VisualParams *vparams) {
    if (! vparams->displayFlags().getShowCollisionModels()) return;

    const std::vector<Vector3> & X = getPos(TVecId::position);

    glDisable(GL_LIGHTING);
    glColor4f(1,0,1,1);
    glBegin(GL_POINTS);
    glPointSize(20);
    for (unsigned i=0;i<m_elements.size();i++) {
        std::shared_ptr<PointElement> elmt = std::dynamic_pointer_cast<PointElement>(m_elements[i]);
        glVertex3dv(X[elmt->m_pid].data());
    }
    glEnd();
}

}
