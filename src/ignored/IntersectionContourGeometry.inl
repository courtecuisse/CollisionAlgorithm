#pragma once

#include <sofa/collisionAlgorithm/geometry/IntersectionContourGeometry.h>

namespace sofa {

namespace collisionAlgorithm {

IntersectionContourGeometry::IntersectionContourGeometry()
: d_planePos(initData(&d_planePos,defaulttype::Vector3(0,0,0),"planePos","Position of the plane"))
, d_planeNormal(initData(&d_planeNormal, defaulttype::Vector3(0,0,1),"planeNormal","Normal of the plane"))
, l_topology(initLink("topology", "Link to topology"))
{}

void IntersectionContourGeometry::prepareDetection() {
    m_elements.clear();

    const helper::ReadAccessor<DataVecCoord> & pos = getState()->read(core::VecCoordId::position());

    m_pointNormal.resize(pos.size());
    for (unsigned p=0;p<pos.size();p++) {
        const core::topology::BaseMeshTopology::TrianglesAroundVertex & tav = l_topology->getTrianglesAroundVertex(p);
        m_pointNormal[p] = defaulttype::Vector3(0,0,0);
        for (unsigned t=0;t<tav.size();t++) {
            core::topology::BaseMeshTopology::Triangle tri = l_topology->getTriangle(tav[t]);

            //Compute Bezier Positions
            defaulttype::Vector3 p0 = pos[tri[0]];
            defaulttype::Vector3 p1 = pos[tri[1]];
            defaulttype::Vector3 p2 = pos[tri[2]];

            m_pointNormal[p] += (p1 - p0).cross(p2 - p0);;
        }
        m_pointNormal[p].normalize();
    }


    defaulttype::Vector3 pointOnPlane = d_planePos.getValue();
    defaulttype::Vector3 planeNormal = d_planeNormal.getValue();
    planeNormal.normalize();

    double d=dot(planeNormal,pointOnPlane);

    //inspect the plane edge intersection
    for(unsigned i=0;i<l_topology->getNbEdges();i++) {
        const core::topology::BaseMeshTopology::Edge & e = l_topology->getEdge(i);

        defaulttype::Vector3 p1 = pos[e[0]];
        defaulttype::Vector3 p2 = pos[e[1]];

        double alpha=1.0 - (d-dot(planeNormal,p1))/(dot(planeNormal,p2-p1));

        if (alpha>=0 && alpha<=1) {
            m_elements.push_back(IntersectionContourElement::createElement(this,e[0],e[1],alpha,1.0-alpha));
        }
    }
}

void IntersectionContourGeometry::draw(const core::visual::VisualParams * vparams) {
    if (! vparams->displayFlags().getShowCollisionModels()) return;

    defaulttype::Vector3 P = d_planePos.getValue();

    defaulttype::Vector3 T(1,0,0);
    defaulttype::Vector3 Z = d_planeNormal.getValue();
    Z.normalize();

    if (fabs(dot(T,Z))>0.9999) T = defaulttype::Vector3(0,1,0);

    defaulttype::Vector3 X = cross(T,Z);
    defaulttype::Vector3 Y = cross(X,Z);

    defaulttype::BoundingBox bbox;
    const helper::ReadAccessor<DataVecCoord> & pos = getState()->read(core::VecCoordId::position());
    for (unsigned i=0;i<pos.size();i++) bbox.include(pos[i]);


    defaulttype::Vector3 min = defaulttype::Vector3(bbox.minBBoxPtr());
    defaulttype::Vector3 max = defaulttype::Vector3(bbox.maxBBoxPtr());

    defaulttype::Vector3 C = (min+max) * 0.5;
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

}
