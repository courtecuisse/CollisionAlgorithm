#pragma once

#include <sofa/collisionAlgorithm/BaseProximity.h>
#include <sofa/collisionAlgorithm/elements/PointElement.h>

namespace sofa::collisionAlgorithm::toolbox {

class PointToolBox {
public:

    static BaseProximity::SPtr createCenterProximity(BaseElement::SPtr elmt) {
        auto point = elmt->element_cast<PointElement>();
        return point->createProximity();
    }


    static BaseProximity::SPtr project(const type::Vector3 & /*P*/, BaseElement::SPtr elmt) {
        auto point = elmt->element_cast<PointElement>();
        return point->createProximity();
    }

};



}

