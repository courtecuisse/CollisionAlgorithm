#pragma once

#include <sofa/collisionAlgorithm/BaseProximity.h>
#include <sofa/collisionAlgorithm/BaseOperation.h>
#include <sofa/collisionAlgorithm/elements/PointElement.h>

namespace sofa::collisionAlgorithm::Operations {

class ProjectOperation : public GenericOperation<ProjectOperation, std::function<BaseProximity::SPtr(type::Vector3 , BaseElement::SPtr)> > {
public:

    using Inherit = GenericOperation;

    Inherit::FUNC getDefault() const override {
        return [=](type::Vector3 , BaseElement::SPtr elmt) -> BaseProximity::SPtr {
            std::cerr << "ERROR no ProjectOperation registered for "  << typeid(elmt.get()).name() << std::endl;
            return NULL;
        };
    }
};

}

