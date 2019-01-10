#pragma once

#include <sofa/core/topology/BaseMeshTopology.h>
#include <sofa/collisionAlgorithm/geometry/TriangleGeometry.h>

namespace sofa
{

namespace collisionAlgorithm
{

template<class DataTypes>
class TriangleProximity;

template<class DataTypes>
class PhongTriangleGeometry : public TriangleGeometry<DataTypes> {
public:
    typedef TriangleGeometry<DataTypes> Inherit;
    SOFA_CLASS(SOFA_TEMPLATE(PhongTriangleGeometry,DataTypes),Inherit);

    typedef sofa::core::topology::BaseMeshTopology::Triangle Triangle;
    typedef size_t TriangleID; // to remove once TriangleID has been changed to size_t in BaseMeshTopology
    typedef helper::vector<Triangle> VecTriangles;
    typedef Data<helper::vector<defaulttype::Vector3> > DataVecCoord;

    virtual ~PhongTriangleGeometry() override {}

    virtual void init() override;

    virtual void prepareDetection() override;

    virtual defaulttype::Vector3 getNormal(const TriangleProximity<DataTypes> * prox) const;

    inline const std::vector<defaulttype::Vector3>& pointNormals() const
    {
        return this->m_pointNormal;
    }

protected:
    std::vector<defaulttype::Vector3> m_pointNormal;
    std::vector< std::vector<TriangleID> > m_trianglesAroundVertex;
};


}

}
