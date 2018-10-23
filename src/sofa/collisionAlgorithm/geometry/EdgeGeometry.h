#pragma once

#include <sofa/collisionAlgorithm/geometry/PointGeometry.h>

namespace sofa {

namespace collisionAlgorithm {

class EdgeElement;

class EdgeGeometry : public PointGeometry {
    friend class EdgeElement;
public:
    SOFA_CLASS(EdgeGeometry,BaseGeometry);

    DataLink<core::topology::BaseMeshTopology> d_topology;

    EdgeGeometry()
    : d_topology(initData(&d_topology, "topology", "this")) {}

    static ConstraintProximity::SPtr createProximity(const EdgeElement * elmt,double f1,double f2);

    void prepareDetection();

    void init();

};

}

}
