#pragma once

#include <geometry/PointGeometry.h>

namespace sofa {

namespace collisionAlgorithm {

class EdgeGeometry : public PointGeometry {
    friend class EdgeElement;
public:

    EdgeGeometry() {}

    void prepareDetection();

    void initialize();

};

}

}
