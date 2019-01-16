#pragma once

#include <sofa/collisionAlgorithm/BaseFilter.h>

namespace sofa {

namespace collisionAlgorithm {

class NormalFilter : public BaseFilter {
public:
    SOFA_ABSTRACT_CLASS(BaseFilter, BaseFilter);

    Data<double> d_angle;

    NormalFilter()
    : d_angle(initData(&d_angle, 0.0, "angle", "Angle filter [-1..1]")) {}

    bool accept(BaseProximity::SPtr p1,BaseProximity::SPtr p2) const {
        return dot(p1->getNormal(),-p2->getNormal())>d_angle.getValue();
    }
};

}

}
