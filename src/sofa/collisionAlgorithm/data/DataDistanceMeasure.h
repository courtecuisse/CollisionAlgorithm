#pragma once

#include <sofa/core/objectmodel/BaseObject.h>
#include <sofa/collisionAlgorithm/BaseProximity.h>
#include <sofa/defaulttype/Vec.h>

namespace sofa {

namespace collisionAlgorithm {

class DistanceMeasure {
public :

    typedef std::function<double(BaseProximity::SPtr p1, BaseProximity::SPtr p2)> DistanceFunction;

    DistanceMeasure(DistanceFunction fct = std::bind(&DistanceMeasure::normedDistance, std::placeholders::_1, std::placeholders::_2)) : compute(fct) {}

    DistanceFunction compute;

    friend std::ostream & operator<< (std::ostream & out, DistanceMeasure) {
        return out ;
    }
    friend std::istream & operator>> (std::istream & in, DistanceMeasure) {
        return in ;
    }

protected:
    static double normedDistance (BaseProximity::SPtr p1, BaseProximity::SPtr p2) {
        return (p1->getPosition()-p2->getPosition()).norm() ;
    }
} ;

}

}
