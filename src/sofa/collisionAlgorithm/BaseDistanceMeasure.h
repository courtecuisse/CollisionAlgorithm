#pragma once

#include <sofa/core/objectmodel/BaseObject.h>

#include <sofa/defaulttype/Vec.h>

namespace sofa {

namespace collisionAlgorithm {

class BaseDistanceMeasure {
public :

    typedef std::function<double(defaulttype::Vector3, defaulttype::Vector3)> DistanceFunction;

    BaseDistanceMeasure(
        DistanceFunction fct = std::bind(&normedDistance, std::placeholders::_1, std::placeholders::_2)
    )
        : dfunct (fct)
    {}

    double getDistance (defaulttype::Vector3 P, defaulttype::Vector3 Q) const {
        return dfunct(P, Q) ;
    }

    friend std::ostream & operator<< (std::ostream & out, BaseDistanceMeasure) {
        return out ;
    }
    friend std::istream & operator>> (std::istream & in, BaseDistanceMeasure) {
        return in ;
    }

protected :
    static double normedDistance (defaulttype::Vector3 P, defaulttype::Vector3 Q) {
        return (P-Q).norm() ;
    }

    DistanceFunction dfunct ;

} ;

}

}
