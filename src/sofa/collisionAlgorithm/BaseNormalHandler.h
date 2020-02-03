#pragma once

#include <sofa/collisionAlgorithm/BaseGeometry.h>
#include <sofa/core/collision/Pipeline.h>
#include <sofa/collisionAlgorithm/DataDetectionOutput.h>

namespace sofa {

namespace collisionAlgorithm {

template<class DataTypes, class PROXIMITYDATA>
class TBaseNormalHandler : public BaseNormalHandler<PROXIMITYDATA> {
public:
    SOFA_ABSTRACT_CLASS(SOFA_TEMPLATE2(TBaseNormalHandler,DataTypes,PROXIMITYDATA), SOFA_TEMPLATE(BaseNormalHandler,PROXIMITYDATA));

    core::objectmodel::SingleLink<TBaseNormalHandler<DataTypes,PROXIMITYDATA>, TBaseGeometry<DataTypes,PROXIMITYDATA>, BaseLink::FLAG_STRONGLINK|BaseLink::FLAG_STOREPATH> l_geometry;

    TBaseNormalHandler()
    : l_geometry(initLink("geometry", "link to the geometry")) {
        l_geometry.setPath("@.");
    }

    void init() {
        if (l_geometry == NULL) return;

        l_geometry->setNormalHandler(this);
        l_geometry->addSlave(this);
    }
};

}

}
