#pragma once

#include <sofa/collisionAlgorithm/BaseProximity.h>
#include <sofa/collisionAlgorithm/BaseOperation.h>
#include <sofa/collisionAlgorithm/elements/EdgeElement.h>
#include <sofa/collisionAlgorithm/proximity/MultiProximity.h>

namespace sofa::collisionAlgorithm::Operations {

class CreateCenterProximityOperation : public GenericOperation<CreateCenterProximityOperation,std::function<BaseProximity::SPtr(BaseElement*)> > {
public:

    using Inherit = GenericOperation;


    static BaseProximity::SPtr defaultCreateCenterProximity(BaseElement* elmt){
        std::vector<BaseProximity::SPtr> res;
        elmt->getControlProximities(res);
        return BaseProximity::create<MultiProximity>(res);
    }

    Inherit::FUNC getDefault() const override { return &CreateCenterProximityOperation::defaultCreateCenterProximity; }

};

}
