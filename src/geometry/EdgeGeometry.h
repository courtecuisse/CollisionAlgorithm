#pragma once

#include <geometry/PointGeometry.h>

namespace collisionAlgorithm {

class EdgeGeometry : public PointGeometry {
    friend class EdgeElement;
public:

    EdgeGeometry() {}

    void prepareDetection();

    void init();

};

}

