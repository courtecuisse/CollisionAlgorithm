#pragma once

#include <sofa/collisionAlgorithm/BaseGeometry.h>
#include <sofa/collisionAlgorithm/proximity/FixedProximity.h>
#include <sofa/core/collision/Pipeline.h>

namespace sofa
{

namespace collisionAlgorithm
{

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

    const PairDetection & operator[](int i) const {
        return m_output[i];
    }

    unsigned size() const {
        return m_output.size();
    }

    inline void add(BaseProximity::SPtr p1,defaulttype::Vector3 & P) {
        m_output.push_back(PairDetection(p1,std::shared_ptr<FixedProximity>(new FixedProximity(P))));
    }

    inline void add(BaseProximity::SPtr p1, BaseProximity::SPtr p2) {
        m_output.push_back(PairDetection(p1,p2));
    }

protected:
    helper::vector< PairDetection > m_output;
};

class BaseGeometryAlgorithm : public core::objectmodel::BaseObject
{
public :

    SOFA_ABSTRACT_CLASS(BaseGeometryAlgorithm, core::objectmodel::BaseObject);

    class BaseFilter : public sofa::core::objectmodel::BaseObject {
    public:
        SOFA_ABSTRACT_CLASS(BaseFilter, sofa::core::objectmodel::BaseObject);

        virtual bool accept(const BaseProximity::SPtr & p1,const BaseProximity::SPtr & p2) const = 0;
    };

    BaseGeometryAlgorithm()
    : l_filters(initLink("filters","list of filters")) {}

    virtual ~BaseGeometryAlgorithm() override {}

    bool acceptFilter(const BaseProximity::SPtr & pfrom,const BaseProximity::SPtr & pdest) const {
        for (auto itfilter = l_filters.begin();itfilter != l_filters.end();itfilter++) {
            const BaseFilter * filter = (*itfilter);
            if (filter == NULL) continue;
            if (! filter->accept(pdest,pfrom)) return false;
        }
        return true;
    }

    virtual void doDetection(const BaseGeometry * from, const BaseGeometry * dst, DetectionOutput & output) = 0;

protected:
    core::objectmodel::MultiLink<BaseGeometryAlgorithm,BaseFilter,BaseLink::FLAG_STRONGLINK|BaseLink::FLAG_STOREPATH> l_filters;
};

}

}
