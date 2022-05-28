#if 0

#pragma once

#include <sofa/collisionAlgorithm/BaseAlgorithm.h>

namespace sofa {

namespace collisionAlgorithm {

/*!
 * \brief The NormalFilter class
 * accepts proximities which angle is superior to a specified value (angle parameter)
 */
class NormalFilter : public BaseAlgorithm::BaseFilter {
public:
    SOFA_ABSTRACT_CLASS(BaseFilter, BaseFilter);

    Data<type::Vec2f> d_angle;

    NormalFilter()
    : d_angle(initData(&d_angle, type::Vec2f(-1.0,1.0), "angle", "Angle filter [-1..1]x[-1..1]")) {}

    bool accept(const BaseProximity::SPtr & p1,const BaseProximity::SPtr & p2) const {
        double angle = dot(p1->getNormal(),p2->getNormal());

        return (angle>=d_angle.getValue()[0] && angle <= d_angle.getValue()[1]);
    }
};

}

}

#endif
