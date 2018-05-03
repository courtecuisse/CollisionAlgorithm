#pragma once

#include <BaseGeometry.h>

namespace collisionAlgorithm {

class IntersectionContourGeometry : public BaseGeometry {
public:
    Data<Vector3> d_planePos;
    Data<Vector3> d_planeNormal;
    Port<Topology,REQUIRED> p_topology;

    IntersectionContourGeometry();

    void prepareDetection();

    void draw(const VisualParams * vparams);

    ReadAccessor<Vector3> read(VecID v) {
        return p_topology->p_state->read(v);
    }

    std::vector<Vector3> m_pointNormal;
};

}
