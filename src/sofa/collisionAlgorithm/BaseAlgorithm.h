#pragma once

#include <sofa/collisionAlgorithm/BaseGeometry.h>
#include <sofa/collisionAlgorithm/proximity/FixedProximity.h>
#include <sofa/core/collision/Pipeline.h>

namespace sofa
{

namespace collisionAlgorithm
{

class BaseFilter : public sofa::core::objectmodel::BaseObject {
public:
    SOFA_ABSTRACT_CLASS(BaseFilter, sofa::core::objectmodel::BaseObject);

    virtual bool accept(const BaseProximity::SPtr & p1,const BaseProximity::SPtr & p2) const = 0;
};

typedef std::pair<BaseProximity::SPtr,BaseProximity::SPtr> PairDetection;

class BaseAlgorithm : public core::objectmodel::BaseObject
{
public :

    SOFA_ABSTRACT_CLASS(BaseAlgorithm, core::objectmodel::BaseObject);

    BaseAlgorithm()
    : l_filters(initLink("filters","list of filters")) {}

    bool acceptFilter(const BaseProximity::SPtr & pfrom,const BaseProximity::SPtr & pdest) const {
        for (auto itfilter = l_filters.begin();itfilter != l_filters.end();itfilter++) {
            const BaseFilter * filter = (*itfilter);
            if (filter == NULL) continue;
            if (! filter->accept(pdest,pfrom)) return false;
        }
        return true;
    }

    virtual void processAlgorithm(const BaseGeometry * g1, const BaseGeometry * g2, helper::vector< PairDetection > & output) = 0;

protected:
    core::objectmodel::MultiLink<BaseAlgorithm,BaseFilter,BaseLink::FLAG_STRONGLINK|BaseLink::FLAG_STOREPATH> l_filters;

};

}

}
