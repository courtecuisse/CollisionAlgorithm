#pragma once

#include <sofa/collisionAlgorithm/geometry/TriangleGeometry.h>

namespace sofa
{

namespace collisionAlgorithm
{


template<class DataTypes>
class PhongTriangleGeometry : public TriangleGeometry<DataTypes> {
public:
    typedef DataTypes TDataTypes;
    typedef TriangleGeometry<DataTypes> Inherit;
    typedef PhongTriangleGeometry<DataTypes> GEOMETRY;
    typedef typename DataTypes::VecCoord VecCoord;
    typedef core::objectmodel::Data< VecCoord >        DataVecCoord;
    typedef size_t TriangleID;
    typedef sofa::core::topology::BaseMeshTopology::Triangle Triangle;
    typedef helper::vector<Triangle> VecTriangles;

    SOFA_CLASS(GEOMETRY,Inherit);

    inline BaseElementIterator::UPtr begin(unsigned eid = 0) const override {
        return DefaultElementIterator<TriangleProximity>::create(this,this->l_topology->getTriangles(),eid);
    }

    inline defaulttype::Vector3 getNormal(const TriangleProximity & data) const {
        return m_point_normals[data.m_p0] * data.m_f0 +
               m_point_normals[data.m_p1] * data.m_f1 +
               m_point_normals[data.m_p2] * data.m_f2;
    }

    virtual void prepareDetection() override {
        Inherit::prepareDetection();

        m_point_normals.resize(this->l_topology->getNbPoints());

        for (size_t p=0;p<m_point_normals.size();p++)
        {
            const sofa::core::topology::BaseMeshTopology::TrianglesAroundVertex & tav = this->l_topology->getTrianglesAroundVertex(p);
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
