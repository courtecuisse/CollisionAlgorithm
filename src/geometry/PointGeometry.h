#pragma once

#include <BaseGeometry.h>

namespace collisionAlgorithm {

class PointGeometry : public BaseGeometry {
    friend class PointElement;

public:
    Port<Topology,REQUIRED> p_topology;

    PointGeometry()
    : p_topology("topology",LEFT,this) {}


    void prepareDetection();

    void init();

    void draw(const VisualParams *vparams);

    inline ReadAccessor<Vector3> read(VecID v) {
        return p_topology->p_state->read(v);
    }

};

}
