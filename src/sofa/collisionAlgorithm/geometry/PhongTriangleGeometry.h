#pragma once

#include <sofa/core/topology/BaseMeshTopology.h>
#include <sofa/collisionAlgorithm/container/DataPhongTriangleContainer.h>

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
    typedef Data<helper::vector<defaulttype::Vector3> > DataVecCoord;

    SOFA_CLASS(GEOMETRY,Inherit);

    DataPhongTriangleContainer<GEOMETRY> d_triangles;

    PhongTriangleGeometry()
    : d_triangles(initData(&d_triangles, "triangles", "Vector of Triangles")){}

    virtual ~PhongTriangleGeometry() override {}

};


}

}
