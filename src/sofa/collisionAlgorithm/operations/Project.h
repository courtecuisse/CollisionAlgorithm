#pragma once

#include <sofa/collisionAlgorithm/BaseProximity.h>
#include <sofa/collisionAlgorithm/BaseOperation.h>
#include <sofa/collisionAlgorithm/elements/PointElement.h>

namespace sofa::collisionAlgorithm::Operations {

typedef std::function<BaseProximity::SPtr(BaseProximity::SPtr, BaseElement::SPtr)> Project_FUNC;

class Project : public GenericOperation<Project_FUNC> {
public:

    static BaseProximity::SPtr s_default(BaseProximity::SPtr, BaseElement::SPtr){
        return NULL;
    }

    Project_FUNC getDefault() const override { return &Project::s_default; }
};

}

