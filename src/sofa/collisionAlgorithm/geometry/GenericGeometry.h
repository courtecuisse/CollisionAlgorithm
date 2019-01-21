#pragma once

#include <sofa/collisionAlgorithm/BaseGeometry.h>
#include <sofa/collisionAlgorithm/BaseElementContainer.h>
#include <sofa/collisionAlgorithm/container/DataTriangleContainer.h>

namespace sofa
{

namespace collisionAlgorithm
{

template<class DataTypes>
class GenericGeometry : public TBaseGeometry<DataTypes> {
public:
    typedef DataTypes TDataTypes;
    typedef TBaseGeometry<DataTypes> Inherit;
    typedef GenericGeometry<DataTypes> GEOMETRY;
    typedef typename DataTypes::Coord Coord;
    typedef typename DataTypes::VecCoord VecCoord;
    typedef Data<VecCoord> DataVecCoord;
    typedef sofa::core::topology::BaseMeshTopology::Triangle Triangle;
    typedef size_t TriangleID; // to remove once TriangleID has been changed to size_t in BaseMeshTopology
    typedef helper::vector<Triangle> VecTriangles;

    SOFA_CLASS(GEOMETRY,Inherit);

    DataElemntContainer<sofa::core::topology::BaseMeshTopology::Edge> d_edges;
    DataElemntContainer<sofa::core::topology::BaseMeshTopology::Triangle> d_triangles;
    DataElemntContainer<sofa::core::topology::BaseMeshTopology::Quad> d_quads;

    GenericGeometry()
    : d_edges(initData(&d_edges, "edges", "Vector of Edges"))
    , d_triangles(initData(&d_triangles, "triangles", "Vector of Triangles"))
    , d_quads(initData(&d_quads, "quads", "Vector of Quads")){}

};


}

}
