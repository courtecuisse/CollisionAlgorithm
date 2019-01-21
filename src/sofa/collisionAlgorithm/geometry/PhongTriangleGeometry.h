#pragma once

#include <sofa/core/topology/BaseMeshTopology.h>
#include <sofa/collisionAlgorithm/elements/DataPhongTriangleElement.h>

namespace sofa
{

namespace collisionAlgorithm
{

template<class DataTypes>
class PhongTriangleGeometry : public TBaseGeometry<DataTypes> {
public:
    typedef DataTypes TDataTypes;
    typedef TBaseGeometry<DataTypes> Inherit;
    typedef PhongTriangleGeometry<DataTypes> GEOMETRY;
    typedef typename DataTypes::Coord Coord;
    typedef sofa::core::topology::BaseMeshTopology::Triangle Triangle;
    typedef size_t TriangleID; // to remove once TriangleID has been changed to size_t in BaseMeshTopology
    typedef helper::vector<Triangle> VecTriangles;
    typedef Data<helper::vector<defaulttype::Vector3> > DataVecCoord;

    SOFA_CLASS(GEOMETRY,Inherit);

    DataPhongTriangleElement<GEOMETRY> d_triangles;

    PhongTriangleGeometry()
    : d_triangles(initData(&d_triangles, "triangles", "Vector of Triangles")){}

    virtual ~PhongTriangleGeometry() override {}

    virtual void init() override;

    virtual void prepareDetection() override;

};


}

}
