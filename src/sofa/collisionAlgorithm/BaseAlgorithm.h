#pragma once

#include <sofa/collisionAlgorithm/BaseGeometry.h>
#include <sofa/collisionAlgorithm/proximity/FixedProximity.h>
#include <sofa/core/collision/Pipeline.h>

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

typedef std::pair<BaseProximity::SPtr,BaseProximity::SPtr> PairDetection;

class DetectionOutput {
public:
    typedef std::pair<BaseProximity::SPtr,BaseProximity::SPtr> PairDetection;

    friend std::ostream& operator<<(std::ostream& i, const DetectionOutput& t)  {
        i << t.m_output.size();
        return i;
    }

    friend std::istream& operator>>(std::istream& i, DetectionOutput& /*t*/) {
        return i;
    }

    void clear() {
        m_output.clear();
    }

    helper::vector<PairDetection>::const_iterator begin() const {
        return m_output.begin();
    }

    helper::vector<PairDetection>::const_iterator end() const {
        return m_output.begin();
    }

    const PairDetection & operator[](int i) const {
        return m_output[i];
    }

    unsigned size() const {
        return m_output.size();
    }

    inline void add(BaseProximity::SPtr p1,defaulttype::Vector3 & P) {
        if (p1 == NULL) return;
        m_output.push_back(PairDetection(p1,std::shared_ptr<FixedProximity>(new FixedProximity(P))));
    }

    inline void add(BaseProximity::SPtr p1, BaseProximity::SPtr p2) {
        if (p1 == NULL) return;
        if (p2 == NULL) return;
        m_output.push_back(PairDetection(p1,p2));
    }

protected:
    helper::vector< PairDetection > m_output;
};

/*!
 * \class BaseAlgorithm
 * \brief The BaseAlgorithm abstract class defines an interface of
 * algorithms to be wrapped in sofa components
 */
class BaseAlgorithm : public core::collision::Pipeline
{
public :

    SOFA_ABSTRACT_CLASS(BaseAlgorithm, core::objectmodel::BaseObject);

    /*!
     * \brief BaseAlgorithm Constructor
     */
    BaseAlgorithm()
    : l_filters(initLink("filters","list of filters")) {}

    /*!
     * \brief acceptFilter loops through all filters linked to the component
     * \param pfrom : source proximity
     * \param pdest : destination proximity
     * \return True if all filters accept proximities, otherwise False
     */
    bool acceptFilter(const BaseProximity::SPtr & pfrom,const BaseProximity::SPtr & pdest) const {
        for (auto itfilter = l_filters.begin();itfilter != l_filters.end();itfilter++) {
            const BaseFilter * filter = (*itfilter);
            if (filter == NULL) continue;
            if (! filter->accept(pdest,pfrom)) return false;
        }
        return true;
    }

    virtual void doDetection() = 0;

    core::objectmodel::MultiLink<BaseAlgorithm,BaseFilter,BaseLink::FLAG_STRONGLINK|BaseLink::FLAG_STOREPATH> l_filters;
protected:

    virtual void reset() override {}

    virtual void doCollisionReset() override {}

    virtual void doCollisionDetection(const sofa::helper::vector<core::CollisionModel*>& /*collisionModels*/) override {}

    virtual void doCollisionResponse() override {}

    virtual std::set< std::string > getResponseList() const override {
        std::set< std::string > res;
        return res;
    }

    void computeCollisionDetection() {
        doDetection();
    }

    void computeCollisionReset() {}

    void computeCollisionResponse()  {}
};

}

}
