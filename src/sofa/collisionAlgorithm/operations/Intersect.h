#pragma once

#include <sofa/collisionAlgorithm/BaseProximity.h>
#include <sofa/collisionAlgorithm/BaseOperation.h>
#include <sofa/collisionAlgorithm/BaseElement.h>

namespace sofa::collisionAlgorithm::Operations {

class IntersectOperation : public GenericOperation2<IntersectOperation, std::function<BaseElement::SPtr (BaseElement* , BaseElement*)> > {
public:

    using Inherit = GenericOperation2;

    Inherit::FUNC getDefault() const override {
        return [=](BaseElement* e1, BaseElement* e2) -> BaseElement::SPtr {
            std::cerr << "ERROR no IntersectOperation registered for "  << e1->name() << "," << e2->name() << std::endl;
            return NULL;
        };
    }
};

}

