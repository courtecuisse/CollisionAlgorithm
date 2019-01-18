#pragma once

#include <sofa/collisionAlgorithm/geometry/PhongTriangleGeometry.h>
#include <sofa/collisionAlgorithm/proximity/TriangleProximity.h>
#include <sofa/collisionAlgorithm/iterators/DefaultElementIterator.h>

namespace sofa
{

namespace collisionAlgorithm
{

template<class DataTypes>
BaseElementIterator::UPtr PhongTriangleGeometry<DataTypes>::begin(unsigned eid) const {
    return DefaultElementIterator<GEOMETRY, TriangleProximity<GEOMETRY> >::create(this, this->d_triangles.getValue(), eid);
}

template<class DataTypes>
void PhongTriangleGeometry<DataTypes>::prepareDetection() {
    TriangleGeometry<DataTypes>::prepareDetection();

    m_point_normals.resize(m_trianglesAroundVertex.size());

    for (size_t p=0;p<m_trianglesAroundVertex.size();p++)
    {
        const std::vector<TriangleID> & tav = m_trianglesAroundVertex[p];
        m_point_normals[p] = defaulttype::Vector3(0,0,0);
        for (size_t t=0;t<tav.size();t++)
        {
            m_point_normals[p] += this->m_triangle_normals[tav[t]];
        }
        m_point_normals[p].normalize();

    }
}

template<class DataTypes>
void PhongTriangleGeometry<DataTypes>::init() {
    TriangleGeometry<DataTypes>::init();

    //store triangles around vertex information
    const VecTriangles& triangles = this->d_triangles.getValue();
    const helper::ReadAccessor<DataVecCoord> & pos = this->l_state->read(core::VecCoordId::position());
    m_trianglesAroundVertex.resize(pos.size());
    for (size_t i = 0; i < triangles.size(); ++i)
    {
        // adding edge i in the edge shell of both points
        for (size_t j=0; j<3; ++j)
            m_trianglesAroundVertex[triangles[i][j]].push_back(i);
    }
}

}

}
