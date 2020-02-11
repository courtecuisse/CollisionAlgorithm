#pragma once

#include <sofa/collisionAlgorithm/BaseNormalHandler.h>

namespace sofa {

namespace collisionAlgorithm {


template<class GEOMETRY>
class PhongTriangleNormalHandler : public TBaseNormalHandler<GEOMETRY> {
public:
    typedef typename GEOMETRY::VecCoord VecCoord;
    typedef typename GEOMETRY::DataVecCoord DataVecCoord;
    typedef typename GEOMETRY::PROXIMITYDATA PROXIMITYDATA;
    typedef typename GEOMETRY::VecTriangles VecTriangles;
    typedef TBaseNormalHandler<GEOMETRY> Inherit;

    SOFA_CLASS(SOFA_TEMPLATE(PhongTriangleNormalHandler,GEOMETRY), Inherit);

    defaulttype::Vector3 computeNormal(const PROXIMITYDATA & data) const override {
        return m_point_normals[data.m_p0] * data.m_f0 +
               m_point_normals[data.m_p1] * data.m_f1 +
               m_point_normals[data.m_p2] * data.m_f2;
    }

    void updateNormals() override {
        m_point_normals.resize(this->l_geometry->l_topology->getNbPoints());

        auto tvecinfo = this->l_geometry->getTriangleInfo();

        m_triangle_normals.clear();
        for (size_t t=0;t<tvecinfo.size();t++) {
            m_triangle_normals.push_back(cross(tvecinfo[t].ax2,tvecinfo[t].ax1));
        }

        for (size_t p=0;p<m_point_normals.size();p++) {
            const sofa::core::topology::BaseMeshTopology::TrianglesAroundVertex & tav = this->l_geometry->l_topology->getTrianglesAroundVertex(p);
            m_point_normals[p] = defaulttype::Vector3(0,0,0);
            for (size_t t=0;t<tav.size();t++) {
                m_point_normals[p] += m_triangle_normals[tav[t]];
            }
            m_point_normals[p].normalize();
        }
    }

protected:
    helper::vector<defaulttype::Vector3> m_triangle_normals;
    helper::vector<defaulttype::Vector3> m_point_normals;

};


}

}
