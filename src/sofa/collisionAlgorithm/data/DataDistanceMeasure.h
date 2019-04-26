#pragma once

#include <sofa/core/objectmodel/BaseObject.h>
#include <sofa/collisionAlgorithm/BaseProximity.h>
#include <sofa/defaulttype/Vec.h>
#include <sofa/core/objectmodel/Data.h>
#include <sofa/collisionAlgorithm/data/DataDetectionOutput.h>

namespace sofa {

namespace collisionAlgorithm {

class DistanceMeasure {
public :

    typedef std::function<double(const collisionAlgorithm::PairDetection & p)> DistanceFunction;

    friend class sofa::core::objectmodel::DataValue<DistanceMeasure, true>;
    friend class sofa::core::objectmodel::DataValue<DistanceMeasure, false>;
    template<class T> friend class sofa::core::objectmodel::Data<T>::InitData;

    DistanceMeasure(DistanceFunction fct) : compute(fct) {}

    DistanceFunction compute;

    friend std::ostream & operator<< (std::ostream & out, DistanceMeasure) {
        return out ;
    }
    friend std::istream & operator>> (std::istream & in, DistanceMeasure) {
        return in ;
    }

protected:
    // this is not allow to create this data without providing the functor
    DistanceMeasure() {}
} ;

}

}
