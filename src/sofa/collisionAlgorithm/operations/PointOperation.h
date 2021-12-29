#pragma once

#include <sofa/collisionAlgorithm/BaseProximity.h>
#include <sofa/collisionAlgorithm/BaseOperation.h>
#include <sofa/collisionAlgorithm/elements/PointElement.h>

namespace sofa::collisionAlgorithm::Operations {

class PointOperation : public BaseOperation {
public:

    static BaseOperation * getOperation() {
        static PointOperation s_op;
        return &s_op;
    }

    static BaseProximity::SPtr createCenterProximity(BaseElement::SPtr elmt) {
        auto point = elmt->cast<PointElement>();
        return point->createProximity();
    }


    static BaseProximity::SPtr project(const BaseProximity::SPtr & /*P*/, BaseElement::SPtr elmt) {
        auto point = elmt->cast<PointElement>();
        return point->createProximity();
    }

protected:
    PointOperation() {
        CreateCenterProximity::register_func(getOperation(),&PointOperation::createCenterProximity);

        Project::register_func(getOperation(),&PointOperation::project);
    }

};



}

