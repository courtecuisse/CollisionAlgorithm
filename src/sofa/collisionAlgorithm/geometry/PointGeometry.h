#pragma once

#include <sofa/core/topology/BaseMeshTopology.h>
#include <sofa/collisionAlgorithm/BaseGeometry.h>

namespace sofa {

namespace collisionAlgorithm {

class PointElement;

class PointGeometry : public BaseGeometry {
    friend class PointElement;
public:
    SOFA_CLASS(PointGeometry,BaseGeometry);

    static ConstraintProximity::SPtr createProximity(const PointElement * elmt);

    virtual void prepareDetection();

    virtual void init();

};

}

}
