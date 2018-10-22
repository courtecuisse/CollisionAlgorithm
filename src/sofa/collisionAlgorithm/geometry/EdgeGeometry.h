#pragma once

#include <sofa/collisionAlgorithm/geometry/PointGeometry.h>

namespace sofa {

namespace collisionAlgorithm {

class EdgeElement;

class EdgeGeometry : public PointGeometry {
    friend class EdgeElement;
public:
    SOFA_CLASS(EdgeGeometry,BaseGeometry);

    EdgeGeometry() {}

    static ConstraintProximity::SPtr createProximity(const EdgeElement * elmt,double f1,double f2);

    void prepareDetection();

    void init();

};

}

}
