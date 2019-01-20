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

class BaseFilter;

class BaseGeometryAlgorithm : public core::collision::Pipeline
{
public :

    SOFA_ABSTRACT_CLASS(BaseGeometryAlgorithm, core::collision::Pipeline);

    virtual ~BaseGeometryAlgorithm() override {}

    virtual void computeCollisionReset() = 0;

    virtual void computeCollisionDetection() = 0;

    void registerFilter(BaseFilter * filter) {
        m_filters.insert(filter);
    }

    void unregisterFilter(BaseFilter * filter) {
        m_filters.erase(filter);
    }

    bool acceptFilter(const BaseProximity::SPtr & pfrom,const BaseProximity::SPtr & pdest) const;


    bool findDataLinkDest(BaseDataElmt *& ptr, const std::string& path, const core::objectmodel::BaseLink* link)
    {
        core::objectmodel::BaseData* base = NULL;
        if (!this->getContext()->findDataLinkDest(base, path, link)) return false;
        ptr = dynamic_cast<BaseDataElmt*>(base);
        return (ptr != NULL);
    }

private:
    void reset() {}

    virtual void computeCollisionResponse() override {
        computeCollisionReset();
        computeCollisionDetection();
    }

    virtual std::set< std::string > getResponseList() const override
    {
        std::set< std::string > res;
        return res;
    }

    virtual void doCollisionReset() override {}

    virtual void doCollisionDetection(const sofa::helper::vector<core::CollisionModel*>& /*collisionModels*/) override {}

    virtual void doCollisionResponse() override {}

protected:
    std::set<BaseFilter*> m_filters;

};

}

}
