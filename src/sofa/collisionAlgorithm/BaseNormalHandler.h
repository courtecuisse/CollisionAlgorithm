#pragma once

#include <sofa/collisionAlgorithm/BaseGeometry.h>

namespace sofa {

namespace collisionAlgorithm {

template<class GEOMETRY>
class TBaseNormalHandler : public sofa::core::objectmodel::BaseObject, public BaseNormalHandler<typename GEOMETRY::PROXIMITYDATA> {
public:

    SOFA_ABSTRACT_CLASS(SOFA_TEMPLATE(TBaseNormalHandler,GEOMETRY), sofa::core::objectmodel::BaseObject);

    core::objectmodel::SingleLink<TBaseNormalHandler<GEOMETRY>, GEOMETRY, BaseLink::FLAG_STRONGLINK|BaseLink::FLAG_STOREPATH> l_geometry;

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
