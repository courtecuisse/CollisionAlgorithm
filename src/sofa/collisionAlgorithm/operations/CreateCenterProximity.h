#pragma once

#include <sofa/collisionAlgorithm/BaseProximity.h>
#include <sofa/collisionAlgorithm/BaseOperation.h>
#include <sofa/collisionAlgorithm/elements/EdgeElement.h>

namespace sofa::collisionAlgorithm::Operations {

class CreateCenterProximityOperation : public GenericOperation<CreateCenterProximityOperation,std::function<BaseProximity::SPtr(BaseElement::SPtr)> > {
public:

    using Inherit = GenericOperation;

    Inherit::FUNC getDefault() const override {
        return [=](BaseElement::SPtr) -> BaseProximity::SPtr {
            return NULL;
        };
    }

};

}

