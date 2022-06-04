#pragma once

#include <sofa/collisionAlgorithm/BaseProximity.h>
#include <sofa/collisionAlgorithm/BaseOperation.h>
#include <sofa/collisionAlgorithm/elements/PointElement.h>

namespace sofa::collisionAlgorithm::Operations {



class ProjectOperation : public GenericOperation<ProjectOperation,//Type of the operation
                                                 BaseProximity::SPtr,//Default return type
                                                 const type::Vector3 & , BaseElement::SPtr//Parameters
                                                 > {
public:

    BaseProximity::SPtr defaultFunc(const type::Vector3 & , BaseElement::SPtr ) const override {
        return NULL;
    }

    void notFound(const std::type_info & id) const override {
        std::cerr << "ERROR the operation ProjectOperation is not registered with for type = " << sofa::helper::NameDecoder::decodeFullName(id) << std::endl;
    }

};

}

