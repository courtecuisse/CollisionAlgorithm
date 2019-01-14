#pragma once

#include <sofa/collisionAlgorithm/geometry/PhongTriangleGeometry.h>
#include <sofa/collisionAlgorithm/proximity/TriangleProximity.h>

namespace sofa
{

namespace collisionAlgorithm
{

template<class DataTypes>
defaulttype::Vector3 PhongTriangleGeometry<DataTypes>::getNormal(const TriangleProximity<DataTypes> *prox) const {
    return m_pointNormal[prox->getPid()[0]] * prox->getFactors()[0] +
           m_pointNormal[prox->getPid()[1]] * prox->getFactors()[1] +
           m_pointNormal[prox->getPid()[2]] * prox->getFactors()[2];
}

template<class DataTypes>
void PhongTriangleGeometry<DataTypes>::prepareDetection() {
    TriangleGeometry<DataTypes>::prepareDetection();

    const VecTriangles& triangles = this->d_triangles.getValue();

    const helper::ReadAccessor<DataVecCoord> & pos = this->l_state->read(core::VecCoordId::position());

    m_pointNormal.resize(pos.size());

    if(triangles.size() > 0)
    {
        m_pointNormal.resize(pos.size());

        for (size_t p=0;p<pos.size();p++)
        {
            const std::vector<TriangleID> & tav = m_trianglesAroundVertex[p];
            m_pointNormal[p] = defaulttype::Vector3(0,0,0);
            for (size_t t=0;t<tav.size();t++)
            {
                m_pointNormal[p] += this->m_triangle_info[tav[t]].tn;
            }
            m_pointNormal[p].normalize();

        }
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
