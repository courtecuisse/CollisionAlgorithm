#pragma once

#include <sofa/collisionAlgorithm/BaseDistanceMeasure.h>

namespace sofa {

namespace collisionAlgorithm {

class Norm3Measure : public core::objectmodel::BaseObject {
public :
    SOFA_CLASS(Norm3Measure, core::objectmodel::BaseObject) ;

    Data<BaseDistanceMeasure> d_distance ;

    Norm3Measure ()
        : d_distance(
              initData(
                  &d_distance,
                  BaseDistanceMeasure(std::bind(&getNormedDistance, std::placeholders::_1, std::placeholders::_2)),
                  "distance",
                  "distance measure data"))
    {}

protected :
    static double getNormedDistance (defaulttype::Vec3 P, defaulttype::Vec3 Q) {
        return (P-Q).norm() ;
    }

} ;


}

}
