#pragma once

#include <sofa/collisionAlgorithm/BaseProximity.h>
#include <sofa/collisionAlgorithm/BaseOperation.h>
#include <sofa/collisionAlgorithm/elements/EdgeElement.h>

namespace sofa::collisionAlgorithm::Operations {

typedef std::function<BaseProximity::SPtr(BaseElement::SPtr)> CreateCenterProximity_FUNC;

class CreateCenterProximity : public GenericOperation<CreateCenterProximity_FUNC> {
public:

    static BaseProximity::SPtr s_default(BaseElement::SPtr){
        return NULL;
    }

    CreateCenterProximity_FUNC getDefault() const override { return &CreateCenterProximity::s_default; }

};

}

