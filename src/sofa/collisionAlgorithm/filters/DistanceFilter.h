#pragma once

#include <sofa/collisionAlgorithm/BaseGeometryAlgorithm.h>

namespace sofa {

namespace collisionAlgorithm {

class DistanceFilter : public BaseGeometryAlgorithm::BaseFilter {
public:
    SOFA_ABSTRACT_CLASS(BaseFilter, BaseFilter);

    Data<double> d_distance;

    DistanceFilter()
     : d_distance(initData(&d_distance, std::numeric_limits<double>::max(), "distance", "Min distance")) {}

    bool accept(const BaseProximity::SPtr & p1,const BaseProximity::SPtr & p2) const {
        return (p1->getPosition()-p2->getPosition()).norm()<d_distance.getValue();
    }
};

}

}
