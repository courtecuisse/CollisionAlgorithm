#pragma once

#include <sofa/collisionAlgorithm/BaseAlgorithm.h>

namespace sofa {

namespace collisionAlgorithm {

/*!
 * \brief The NormalFilter class
 * accepts proximities which angle is superior to a specified value (angle parameter)
 */
class NormalFilter : public BaseFilter {
public:
    SOFA_ABSTRACT_CLASS(BaseFilter, BaseFilter);

    Data<double> d_angle;

    NormalFilter()
    : d_angle(initData(&d_angle, 0.0, "angle", "Angle filter [-1..1]")) {}

    bool accept(const BaseProximity::SPtr & p1,const BaseProximity::SPtr & p2) const {
        return dot(p1->getNormal(),p2->getNormal())>d_angle.getValue();
    }
};

}

}
