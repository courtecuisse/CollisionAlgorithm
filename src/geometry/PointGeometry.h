#pragma once

#include <BaseGeometry.h>

namespace collisionAlgorithm {

class PointGeometry : public BaseGeometry {
    friend class PointElement;

public:
    Port<Topology,IN> p_topology;
    Data<Vector4> d_color;

    PointGeometry()
    : p_topology("topology",REQUIRED,this)
    , d_color("color", Vector4(1,0,1,1), this){}

    void prepareDetection();

    void init();

    State * getState() {
        return p_topology->p_state();
    }

};

}
