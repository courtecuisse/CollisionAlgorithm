#pragma once

#include <geometry/IntersectionContourGeometry.h>
#include <element/IntersectionContourElement.h>
#include <qopengl.h>

namespace collisionAlgorithm {

IntersectionContourGeometry::IntersectionContourGeometry()
: d_planePos("planePos",Vector3(0,0,0),this)
, d_planeNormal("planeNormal",Vector3(0,0,1),this)
{}

void IntersectionContourGeometry::prepareDetection() {
    m_elements.clear();

    const ReadAccessor<Vector3> & pos = getState()->read(VecCoordId::position());

    m_pointNormal.resize(pos.size());
    for (unsigned p=0;p<pos.size();p++) {
        const Topology::TrianglesAroundVertex & tav = this->p_topology->getTrianglesAroundVertex(p);
        m_pointNormal[p] = Vector3(0,0,0);
        for (unsigned t=0;t<tav.size();t++) {
            Topology::Triangle tri = this->p_topology->getTriangle(tav[t]);

            //Compute Bezier Positions
            Vector3 p0 = pos[tri[0]];
            Vector3 p1 = pos[tri[1]];
            Vector3 p2 = pos[tri[2]];

            m_pointNormal[p] += (p1 - p0).cross(p2 - p0);;
        }
        m_pointNormal[p].normalize();
    }


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

    BoundingBox bbox;
    const ReadAccessor<Vector3> & pos = getState()->read(VecCoordId::position());
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

    BaseGeometry::draw(vparams);
}

}
