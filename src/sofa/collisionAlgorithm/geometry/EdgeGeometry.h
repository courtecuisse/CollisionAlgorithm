#pragma once

#include <sofa/collisionAlgorithm/geometry/PointGeometry.h>

namespace sofa {

namespace collisionAlgorithm {

class EdgeElement;

class EdgeGeometry : public BaseGeometry {
    friend class EdgeElement;
public:
    typedef sofa::core::topology::BaseMeshTopology::Edge Edge;
    typedef helper::vector<Edge> VecEdges;

    SOFA_CLASS(EdgeGeometry,BaseGeometry);

    EdgeGeometry()
        : d_edges(initData(&d_edges, VecEdges(), "edges", "Vector of Edges"))
    {

    }

    BaseProximity::SPtr createProximity(const EdgeElement * elmt,double f1,double f2) const;

    virtual void prepareDetection() override;

    virtual void init() override;

    inline const VecEdges& edges() const { return d_edges.getValue(); }

protected:
    Data<VecEdges> d_edges;

};

}

}
