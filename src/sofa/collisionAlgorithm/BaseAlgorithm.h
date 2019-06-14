#pragma once

#include <sofa/collisionAlgorithm/BaseGeometry.h>
#include <sofa/core/collision/Pipeline.h>
#include <sofa/collisionAlgorithm/data/DataDetectionOutput.h>

namespace sofa
{

namespace collisionAlgorithm
{

/*!
 * \brief The BaseFilter class provides an interface to create proximity filter components
 */
class BaseFilter : public sofa::core::objectmodel::BaseObject {
public:
    SOFA_ABSTRACT_CLASS(BaseFilter, sofa::core::objectmodel::BaseObject);

    /*!
     * \brief accept is a pure virtual method to implement
     * \param p1 : source proximity
     * \param p2 : destination proximity
     * \return bool : if the filter accepted the proximities
     */
    virtual bool accept(const BaseProximity::SPtr & p1,const BaseProximity::SPtr & p2) const = 0;
};

/*!
 * \class BaseAlgorithm
 * \brief The BaseAlgorithm abstract class defines an interface of
 * algorithms to be wrapped in sofa components
 */
class BaseAlgorithm : public sofa::core::objectmodel::BaseObject
{
public :

    SOFA_ABSTRACT_CLASS(BaseAlgorithm, sofa::core::objectmodel::BaseObject);

    core::objectmodel::MultiLink<BaseAlgorithm,BaseFilter,BaseLink::FLAG_STRONGLINK|BaseLink::FLAG_STOREPATH> l_filters;

    /*!
     * \brief BaseAlgorithm Constructor
     */
    BaseAlgorithm()
    : l_filters(initLink("filters","list of filters")){
        this->f_listening.setValue(true);
    }

    /*!
     * \brief acceptFilter loops through all filters linked to the component
     * \param pfrom : source proximity
     * \param pdest : destination proximity
     * \return True if all filters accept proximities, otherwise False
     */
    bool acceptFilter(const BaseProximity::SPtr & pfrom,const BaseProximity::SPtr & pdest) const {
        for (auto itfilter = l_filters.begin();itfilter != l_filters.end();itfilter++) {
            const BaseFilter * filter = (*itfilter).get();
            if (filter == NULL) continue;
            if (! filter->accept(pfrom,pdest)) return false;
        }
        return true;
    }

    void init() {
        for (unsigned i=0;i<l_filters.size();i++) {
            if (l_filters[i] == NULL) continue;
            this->addSlave(l_filters[i]);
        }
    }


protected:

    virtual void doDetection() = 0;

    void handleEvent(sofa::core::objectmodel::Event *event) {
        if (! dynamic_cast<sofa::simulation::CollisionBeginEvent*>(event)) return;

        doDetection();
    }
};

}

}
