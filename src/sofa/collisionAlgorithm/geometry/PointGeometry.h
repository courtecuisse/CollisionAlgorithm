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

    Data<defaulttype::Vector4> d_color;

    PointGeometry()
    : d_color(initData(&d_color, defaulttype::Vector4(1,0,1,1), "color", "Color of the collision model")) {}

    static ConstraintProximity::SPtr createProximity(const PointElement * elmt);

    void prepareDetection();

    void init();

};

}

}
