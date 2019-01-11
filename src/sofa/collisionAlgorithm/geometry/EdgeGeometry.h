#pragma once

#include <sofa/collisionAlgorithm/geometry/PointGeometry.h>

namespace sofa {

namespace collisionAlgorithm {


template<class DataTypes>
class EdgeGeometry : public TBaseGeometry<DataTypes> {
public:
    typedef TBaseGeometry<DataTypes> Inherit;
    SOFA_CLASS(SOFA_TEMPLATE(EdgeGeometry,DataTypes),Inherit);

    typedef sofa::core::topology::BaseMeshTopology::Edge Edge;
    typedef helper::vector<Edge> VecEdges;

    Data<VecEdges> d_edges;

    EdgeGeometry()
    : d_edges(initData(&d_edges, VecEdges(), "edges", "Vector of Edges")) {}

    virtual BaseElement::Iterator begin(unsigned eid = 0) const;

};

}

}
