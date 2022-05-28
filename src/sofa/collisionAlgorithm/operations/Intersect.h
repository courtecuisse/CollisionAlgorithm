#pragma once

#include <sofa/collisionAlgorithm/BaseProximity.h>
#include <sofa/collisionAlgorithm/BaseOperation.h>
#include <sofa/collisionAlgorithm/BaseElement.h>
#include <boost/optional.hpp>

namespace sofa::collisionAlgorithm::Operations {

class IntersectOperation : public GenericOperation2<IntersectOperation, std::function<std::pair<BaseElement::SPtr,BaseElement::SPtr> (BaseElement::SPtr , BaseElement::SPtr)> > {
public:

    using Inherit = GenericOperation2;

    Inherit::FUNC getDefault() const override {
        return [=](BaseElement::SPtr e1, BaseElement::SPtr e2) -> std::pair<BaseElement::SPtr,BaseElement::SPtr> {
            std::cerr << "ERROR no IntersectOperation registered for "  << e1->name() << "," << e2->name() << std::endl;
            return std::pair<BaseElement::SPtr,BaseElement::SPtr>();
        };
    }
};

}

