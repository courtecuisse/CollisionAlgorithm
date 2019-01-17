#pragma once

#include <sofa/collisionAlgorithm/geometry/PointGeometry.h>

namespace sofa {

namespace collisionAlgorithm {


template<class DataTypes>
class EdgeGeometry : public TBaseGeometry<DataTypes> {
public:
    typedef TBaseGeometry<DataTypes> Inherit;
    typedef DataTypes TDataTypes;
    typedef typename DataTypes::Coord Coord;
    typedef typename DataTypes::VecCoord VecCoord;
    typedef Data<VecCoord> DataVecCoord;

    SOFA_CLASS(SOFA_TEMPLATE(EdgeGeometry,DataTypes),Inherit);

    typedef sofa::core::topology::BaseMeshTopology::Edge Edge;
    typedef helper::vector<Edge> VecEdges;

    Data<VecEdges> d_edges;

    EdgeGeometry()
    : d_edges(initData(&d_edges, VecEdges(), "edges", "Vector of Edges")) {}

    virtual BaseElementIterator::UPtr begin(unsigned eid = 0) const;

    virtual BaseProximity::SPtr project(unsigned tid, const defaulttype::Vector3 & P) const;

    virtual BaseProximity::SPtr center(unsigned tid) const;

    virtual defaulttype::BoundingBox getBBox(unsigned tid) const;

};

}

}
