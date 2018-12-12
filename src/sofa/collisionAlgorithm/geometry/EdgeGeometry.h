#pragma once

#include <sofa/collisionAlgorithm/geometry/PointGeometry.h>

namespace sofa {

namespace collisionAlgorithm {

class EdgeElement;

class EdgeGeometry : public PointGeometry {
    friend class EdgeElement;
public:
    SOFA_CLASS(EdgeGeometry,BaseGeometry);

    EdgeGeometry()
    : l_topology(initLink("topology", "Link to topology"))
    {
        l_topology.setPath("@.");
    }

    static ConstraintProximity::SPtr createProximity(const EdgeElement * elmt,double f1,double f2);

    virtual void prepareDetection() override;

    virtual void init() override;

protected:
    core::objectmodel::SingleLink<EdgeGeometry,core::topology::BaseMeshTopology,BaseLink::FLAG_STRONGLINK|BaseLink::FLAG_STOREPATH> l_topology;

};

}

}
