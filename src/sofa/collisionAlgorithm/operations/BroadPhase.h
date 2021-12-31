#pragma once

#include <sofa/collisionAlgorithm/BaseOperation.h>
#include <sofa/collisionAlgorithm/BaseElement.h>
#include <sofa/collisionAlgorithm/BaseProximity.h>

namespace sofa::collisionAlgorithm::Operations {

typedef std::function<BaseElement::Iterator(BaseProximity::SPtr,BaseGeometry *) > BroadPhase_FUNC;

class BroadPhase : public GenericOperation<BroadPhase_FUNC> {
public:

    //By default no broadPhase so we loop over all elements
    static BaseElement::Iterator s_default(BaseProximity::SPtr,BaseGeometry * geo) {
        return geo->begin();
    }

    BroadPhase_FUNC getDefault() const override { return &BroadPhase::s_default; }

};

}

