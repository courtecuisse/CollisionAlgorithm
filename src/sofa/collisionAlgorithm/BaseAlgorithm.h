#pragma once

#include <sofa/collisionAlgorithm/BaseGeometry.h>
#include <sofa/core/collision/Pipeline.h>
#include <sofa/collisionAlgorithm/DataDetectionOutput.h>

namespace sofa
{

namespace collisionAlgorithm
{

/*!
 * \class BaseAlgorithm
 * \brief The BaseAlgorithm abstract class defines an interface of
 * algorithms to be wrapped in sofa components
 */
class BaseAlgorithm : public sofa::core::objectmodel::BaseObject
{
public :

    SOFA_ABSTRACT_CLASS(BaseAlgorithm, sofa::core::objectmodel::BaseObject);

    /*!
     * \brief The BaseFilter class provides an interface to create proximity filter components
     */
    class BaseFilter : public sofa::core::objectmodel::BaseObject {
    public:
        SOFA_ABSTRACT_CLASS(BaseFilter, sofa::core::objectmodel::BaseObject);

        core::objectmodel::SingleLink<BaseFilter,BaseAlgorithm,BaseLink::FLAG_STRONGLINK|BaseLink::FLAG_STOREPATH> l_algo;

        /*!
         * \brief BaseAlgorithm Constructor
         */
        BaseFilter()
        : l_algo(initLink("algo","list of filters")){
            l_algo.setPath("@.");
            this->f_listening.setValue(true);
        }

        void init() override {
            if (l_algo == NULL) return;
            l_algo->addFilter(this);
            l_algo->addSlave(this);
        }

        /*!
         * \brief accept is a pure virtual method to implement
         * \param p1 : source proximity
         * \param p2 : destination proximity
         * \return bool : if the filter accepted the proximities
         */
        virtual bool accept(const BaseProximity::SPtr & p1,const BaseProximity::SPtr & p2) const = 0;
    };

    /*!
     * \brief BaseAlgorithm Constructor
     */
    BaseAlgorithm() {
        this->f_listening.setValue(true);
    }

    /*!
     * \brief acceptFilter loops through all filters linked to the component
     * \param pfrom : source proximity
     * \param pdest : destination proximity
     * \return True if all filters accept proximities, otherwise False
     */
    bool acceptFilter(const PairDetection & d) const {
        for (unsigned i=0;i<m_filters.size();i++) {
            if (! m_filters[i]->accept(d.first,d.second)) return false;
        }
        return true;
    }

    void addFilter(BaseFilter::SPtr f) {
        m_filters.push_back(f);
    }


protected:

    virtual void doDetection() = 0;

    void handleEvent(sofa::core::objectmodel::Event *event) {
        if (! dynamic_cast<sofa::simulation::CollisionBeginEvent*>(event)) return;

        doDetection();
    }

    sofa::type::vector<BaseFilter::SPtr> m_filters;
};

}

}
