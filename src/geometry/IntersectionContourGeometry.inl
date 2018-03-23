#pragma once

#include <geometry/IntersectionContourGeometry.h>
#include <GL/gl.h>

namespace collisionAlgorithm {

/**************************************************************************/
/******************************PROXIMITY***********************************/
/**************************************************************************/

IntersectionContourProximity::IntersectionContourProximity(IntersectionContourElement * elmt) : ConstraintProximity(elmt) {}

Vector3 IntersectionContourProximity::getPosition(TVecId v) const {
    const std::vector<Vector3> & pos = element()->geometry()->getPos(v);
    Vector3 P = pos[element()->m_pid[0]] * element()->m_fact[0];
    Vector3 Q = pos[element()->m_pid[1]] * element()->m_fact[1];

    return (P+Q);
}

Vector3 IntersectionContourProximity::getNormal() const {
//        return m_geo->getPos(ie.p1,vid) * ie.alpha + m_geo->getPos(ie.p2,vid) * (1.0 - ie.alpha);
}

std::map<unsigned,Vector3> IntersectionContourProximity::getContribution(const Vector3 & N) {

}

/**************************************************************************/
/******************************ELEMENT*************************************/
/**************************************************************************/

IntersectionContourElement::IntersectionContourElement(IntersectionContourGeometry *geo, unsigned pid1, unsigned pid2,double f1,double f2) : ConstraintElement(geo) {
    m_pid[0] = pid1;
    m_pid[1] = pid2;

    m_fact[0] = f1;
    m_fact[1] = f2;
}

ConstraintProximityPtr IntersectionContourElement::project(Vector3 /*P*/) {

}

ConstraintProximityPtr IntersectionContourElement::getControlPoint(const int i) {

}

unsigned IntersectionContourElement::getNbControlPoints() {

}


void IntersectionContourElement::draw(const std::vector<Vector3> & X) {
    glBegin(GL_POINTS);
    glPointSize(20);
    glVertex3dv((X[m_pid[0]] * m_fact[0] + X[m_pid[1]] * m_fact[1]).data());
    glEnd();
}

/**************************************************************************/
/******************************GEOMETRY************************************/
/**************************************************************************/

IntersectionContourGeometry::IntersectionContourGeometry()
: d_planePos("planePos",Vector3(0,0,0),this)
, d_planeNormal("planeNormal",Vector3(0,0,1),this)
{}

void IntersectionContourGeometry::prepareDetection() {
    m_elements.clear();

    const std::vector<Vector3> & x = getPos(TVecId::position);

    Vector3 pointOnPlane = d_planePos.getValue();
    Vector3 planeNormal = d_planeNormal.getValue();
    planeNormal.normalize();

    double d=dot(planeNormal,pointOnPlane);

    //inspect the plane edge intersection
    for(unsigned i=0;i<p_topology->getNbEdges();i++) {
        const TEdge & e = p_topology->getEdge(i);

        Vector3 p1 = x[e[0]];
        Vector3 p2 = x[e[1]];

        double alpha=1.0 - (d-dot(planeNormal,p1))/(dot(planeNormal,p2-p1));

        if (alpha>=0 && alpha<=1) {
            m_elements.push_back(std::make_shared<IntersectionContourElement>(this,e[0],e[1],alpha,1.0-alpha));
        }
    }
}

//void IntersectionContourGeometry::draw(const VisualParams * vparams) {
//    if (! vparams->displayFlags().getShowCollisionModels()) return;

//    Vector3 P = d_planePos.getValue();

//    Vector3 T(1,0,0);
//    Vector3 Z = d_planeNormal.getValue();
//    Z.normalize();

//    if (fabs(dot(T,Z))>0.9999) T = Vector3(0,1,0);

//    Vector3 X = cross(T,Z);
//    Vector3 Y = cross(X,Z);

////    std::cout << "P=" << P << std::endl;
////    std::cout << "X=" << X << std::endl;
////    std::cout << "Y=" << Y << std::endl;

//    BoundingBox bbox;
//    p_topology->p_state->computeBBox(bbox);
//    Vector3 C = (Vector3(bbox.min()) + Vector3(bbox.max())) * 0.5;
//    C -= Z*dot(C-P,Z);

//    double norm = bbox.norm();

//    X = X * norm * 0.5;
//    Y = Y * norm * 0.5;
////    std::cout << "BBOX = " << bbox << std::endl;

//    glBegin(GL_LINE_LOOP);
//        glVertex3dv((C-X-Y).data());
//        glVertex3dv((C+X-Y).data());
//        glVertex3dv((C+X+Y).data());
//        glVertex3dv((C-X+Y).data());
//    glEnd();

//    const std::vector<Vector3> & pos = getPos(TVecId::position);
//    for (unsigned i=0;i<m_elements.size();i++) {
//        m_elements[i]->draw(pos);
//    }
//}

}
