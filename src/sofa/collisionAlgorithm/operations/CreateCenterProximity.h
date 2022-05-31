#pragma once

#include <sofa/collisionAlgorithm/BaseProximity.h>
#include <sofa/collisionAlgorithm/BaseOperation.h>
#include <sofa/collisionAlgorithm/elements/EdgeElement.h>
#include <sofa/collisionAlgorithm/proximity/MultiProximity.h>

namespace sofa::collisionAlgorithm::Operations {

class CreateCenterProximityOperation : public GenericOperation<CreateCenterProximityOperation, //type of operation
                                                               BaseProximity::SPtr, //Default return type
                                                               BaseElement::SPtr //Parameters
                                                              > {
public:

    BaseProximity::SPtr defaultFunc(BaseElement::SPtr) const override {
        return NULL;
    }
};

}

