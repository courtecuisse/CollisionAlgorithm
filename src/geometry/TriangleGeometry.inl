#pragma once

#include <geometry/TriangleGeometry.h>
#include <element/TriangleElement.h>

namespace sofa {

namespace collisionAlgorithm {

void TriangleGeometry::prepareDetection() {
    if (m_elements.size() != d_topology->getNbTriangles()) init();

    const helper::ReadAccessor<DataVecCoord> & pos = getState()->read(core::VecCoordId::position());

    m_pointNormal.resize(d_topology->getNbPoints());

    m_triangle_info.resize(d_topology->getTriangles().size());

    for (unsigned t=0;t<d_topology->getNbTriangles();t++) {
        TriangleInfo & tinfo = m_triangle_info[t];

        const core::topology::BaseMeshTopology::Triangle tri = d_topology->getTriangle(t);

        //Compute Bezier Positions
        defaulttype::Vector3 p0 = pos[tri[0]];
        defaulttype::Vector3 p1 = pos[tri[1]];
        defaulttype::Vector3 p2 = pos[tri[2]];

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
        const core::topology::BaseMeshTopology::TrianglesAroundVertex & tav = d_topology->getTrianglesAroundVertex(p);
        m_pointNormal[p] = defaulttype::Vector3(0,0,0);
        for (unsigned t=0;t<tav.size();t++) {
            m_pointNormal[p] += this->m_triangle_info[tav[t]].tn;
        }
        m_pointNormal[p].normalize();
    }
}

void TriangleGeometry::initialize() {
    m_elements.clear();

    for (unsigned i=0;i<d_topology->getNbTriangles();i++) {
        m_elements.push_back(std::make_shared<TriangleElement>(this,i));
    }

    prepareDetection();
}

}

}
