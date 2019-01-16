#pragma once

#include <sofa/collisionAlgorithm/BaseGeometryAlgorithm.h>
#include <sofa/core/BehaviorModel.h>

namespace sofa
{

namespace collisionAlgorithm
{

class BaseFilter : public sofa::core::objectmodel::BaseObject {
public:
    SOFA_ABSTRACT_CLASS(BaseFilter, sofa::core::objectmodel::BaseObject);

    BaseFilter()
    : l_algo(initLink("algo", "link to algorithm")) {}

    ~BaseFilter() {
        l_algo->unregisterFilter(this);
    }

    void init() {
        l_algo->sout << "Register filter " << this->getName() << l_algo->sendl;
        l_algo->registerFilter(this);
    }

    virtual bool accept(BaseProximity::SPtr,BaseProximity::SPtr) const = 0;


    core::objectmodel::SingleLink<BaseFilter,BaseGeometryAlgorithm,BaseLink::FLAG_STRONGLINK|BaseLink::FLAG_STOREPATH|BaseLink::FLAG_DOUBLELINK> l_algo;
};

}

}
