#pragma once

#include <sofa/core/objectmodel/BaseObject.h>
#include <sofa/defaulttype/Vec.h>

namespace sofa {

namespace collisionAlgorithm {

class BaseDistanceMeasure : public core::objectmodel::BaseObject {
public :
    SOFA_ABSTRACT_CLASS(BaseDistanceMeasure, core::objectmodel::BaseObject) ;

    BaseDistanceMeasure() {}

    virtual double computeDistance (defaulttype::Vec3 P, defaulttype::Vec3 Q) {
        return -1.0 ;
    }

} ;

}

}
