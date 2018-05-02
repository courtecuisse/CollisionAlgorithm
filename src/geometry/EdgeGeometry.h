#pragma once

#include <geometry/PointGeometry.h>

namespace collisionAlgorithm {

class EdgeGeometry : public BaseGeometry {
    friend class EdgeElement;
public:
    Port<Topology,REQUIRED> p_topology;

    EdgeGeometry()
    : p_topology("topology",LEFT,this) {}

    void prepareDetection();

    void init();

    void draw(const VisualParams *vparams);

    inline ReadAccessor<Vector3> read(VecID v) {
        return p_topology->p_state->read(v);
    }
};

}

