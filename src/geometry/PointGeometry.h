#pragma once

#include <BaseGeometry.h>

namespace collisionAlgorithm {

class PointGeometry : public BaseGeometry {
    friend class PointElement;

public:
    Data<Vector4> d_color;

    PointGeometry()
    : d_color("color", Vector4(1,0,1,1), this){}

    void prepareDetection();

    void init();

    State * getState() {
        return p_topology->p_state();
    }

protected:
    Topology * m_topology;
};

}
