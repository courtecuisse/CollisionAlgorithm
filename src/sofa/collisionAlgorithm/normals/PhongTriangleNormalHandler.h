#pragma once

#include <sofa/collisionAlgorithm/normals/TriangleNormalHandler.h>

namespace sofa {

namespace collisionAlgorithm {


template<class GEOMETRY>
class PhongTriangleNormalHandler : public TriangleNormalHandler<GEOMETRY> {
public:
    typedef typename GEOMETRY::VecCoord VecCoord;
    typedef typename GEOMETRY::DataVecCoord DataVecCoord;
    typedef typename GEOMETRY::PROXIMITYDATA PROXIMITYDATA;
    typedef typename GEOMETRY::VecTriangles VecTriangles;
    typedef TriangleNormalHandler<GEOMETRY> Inherit;

    SOFA_CLASS(SOFA_TEMPLATE(PhongTriangleNormalHandler,GEOMETRY), Inherit);

    inline defaulttype::Vector3 getNormal(const PROXIMITYDATA & data) const {
        return m_point_normals[data.m_p0] * data.m_f0 +
               m_point_normals[data.m_p1] * data.m_f1 +
               m_point_normals[data.m_p2] * data.m_f2;
    }

    void updateNormals() override {
        Inherit::updateNormals();

        m_point_normals.resize(this->l_geometry->l_topology->getNbPoints());

        for (size_t p=0;p<m_point_normals.size();p++)
        {
            const sofa::core::topology::BaseMeshTopology::TrianglesAroundVertex & tav = this->l_geometry->l_topology->getTrianglesAroundVertex(p);
            m_point_normals[p] = defaulttype::Vector3(0,0,0);
            for (size_t t=0;t<tav.size();t++) {
                m_point_normals[p] += this->m_triangle_normals[tav[t]];
            }
            m_point_normals[p].normalize();
        }
    }

protected:
    helper::vector<defaulttype::Vector3> m_point_normals;

};


}

}
