#pragma once

#include <geometry/IntersectionContourGeometry.h>
#include <GL/gl.h>

namespace collisionAlgorithm {

/**************************************************************************/
/******************************PROXIMITY***********************************/
/**************************************************************************/

IntersectionContourProximity::IntersectionContourProximity(IntersectionContourElement * elmt) : ConstraintProximity(elmt) {}

Vector3 IntersectionContourProximity::getPosition(VecID v) const {
    const ReadAccessor<Vector3> & pos = element()->geometry()->read(v);

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

IntersectionContourElement::IntersectionContourElement(IntersectionContourGeometry *geo, unsigned pid1, unsigned pid2,double f1,double f2) : ConstraintElement(geo,1) {
    m_pid[0] = pid1;
    m_pid[1] = pid2;

    m_fact[0] = f1;
    m_fact[1] = f2;
}

ConstraintProximityPtr IntersectionContourElement::project(Vector3 /*P*/) {
    return getControlPoint(0);
}

ConstraintProximityPtr IntersectionContourElement::getControlPoint(const int /*i*/) {
    return std::make_shared<IntersectionContourProximity>(this);
}

/**************************************************************************/
/******************************GEOMETRY************************************/
/**************************************************************************/

IntersectionContourGeometry::IntersectionContourGeometry()
: d_planePos("planePos",Vector3(0,0,0),this)
, d_planeNormal("planeNormal",Vector3(0,0,1),this)
, p_topology("topology",LEFT,this)
{}

void IntersectionContourGeometry::prepareDetection() {
    m_elements.clear();

    const ReadAccessor<Vector3> & pos = p_topology->p_state->read(VecCoordId::position());

    Vector3 pointOnPlane = d_planePos.getValue();
    Vector3 planeNormal = d_planeNormal.getValue();
    planeNormal.normalize();

    double d=dot(planeNormal,pointOnPlane);

    //inspect the plane edge intersection
    for(unsigned i=0;i<p_topology->getNbEdges();i++) {
        const Topology::Edge & e = p_topology->getEdge(i);

        Vector3 p1 = pos[e[0]];
        Vector3 p2 = pos[e[1]];

        double alpha=1.0 - (d-dot(planeNormal,p1))/(dot(planeNormal,p2-p1));

        if (alpha>=0 && alpha<=1) {
            m_elements.push_back(std::make_shared<IntersectionContourElement>(this,e[0],e[1],alpha,1.0-alpha));
        }
    }
}

void IntersectionContourGeometry::draw(const VisualParams * vparams) {
    if (! vparams->displayFlags().getShowCollisionModels()) return;

    Vector3 P = d_planePos.getValue();

    Vector3 T(1,0,0);
    Vector3 Z = d_planeNormal.getValue();
    Z.normalize();

    if (fabs(dot(T,Z))>0.9999) T = Vector3(0,1,0);

    Vector3 X = cross(T,Z);
    Vector3 Y = cross(X,Z);

//    std::cout << "P=" << P << std::endl;
//    std::cout << "X=" << X << std::endl;
//    std::cout << "Y=" << Y << std::endl;

    BoundingBox bbox;
    const ReadAccessor<Vector3> & pos = p_topology->p_state->read(VecCoordId::position());
    for (unsigned i=0;i<pos.size();i++) bbox.include(pos[i]);


    Vector3 min = Vector3(bbox.minBBoxPtr());
    Vector3 max = Vector3(bbox.maxBBoxPtr());

    Vector3 C = (min+max) * 0.5;
    C -= Z*dot(C-P,Z);

    double norm = (max - min).norm();

    X = X * norm * 0.5;
    Y = Y * norm * 0.5;
//    std::cout << "BBOX = " << bbox << std::endl;

    glBegin(GL_LINE_LOOP);
        glVertex3dv((C-X-Y).data());
        glVertex3dv((C+X-Y).data());
        glVertex3dv((C+X+Y).data());
        glVertex3dv((C-X+Y).data());
    glEnd();

    glBegin(GL_POINTS);
    glPointSize(20);
    for (unsigned i=0;i<m_elements.size();i++) {
        std::shared_ptr<IntersectionContourElement> elmt = std::dynamic_pointer_cast<IntersectionContourElement>(m_elements[i]);
        Vector3 p1 = pos[elmt->m_pid[0]] * elmt->m_fact[0];
        Vector3 p2 = pos[elmt->m_pid[1]] * elmt->m_fact[1];
        glVertex3dv((p1+p2).data());
    }
    glEnd();
}

}
