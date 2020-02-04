#pragma once

#include <sofa/collisionAlgorithm/BaseNormalHandler.h>

namespace sofa {

namespace collisionAlgorithm {

template<class GEOMETRY>
class TriangleNormalHandler : public TBaseNormalHandler<GEOMETRY> {
public:
    typedef typename GEOMETRY::VecCoord VecCoord;
    typedef typename GEOMETRY::DataVecCoord DataVecCoord;
    typedef typename GEOMETRY::PROXIMITYDATA PROXIMITYDATA;
    typedef typename GEOMETRY::VecTriangles VecTriangles;
    typedef TBaseNormalHandler<GEOMETRY> Inherit;

    SOFA_CLASS(SOFA_TEMPLATE(TriangleNormalHandler,GEOMETRY), Inherit);

    void updateNormals() override {
        const VecTriangles& triangles = this->l_geometry->l_topology->getTriangles();

        m_triangle_normals.resize(triangles.size());

        for (size_t t=0 ; t<triangles.size() ; t++) {
            auto tinfo = this->l_geometry->getTriangleInfo(t);
            m_triangle_normals[t] = tinfo.n;

        }
    }

    defaulttype::Vector3 computeNormal(const PROXIMITYDATA & data) const override {
        return m_triangle_normals[data.m_eid];
    }

protected:
    helper::vector<defaulttype::Vector3> m_triangle_normals;
};

}

}
