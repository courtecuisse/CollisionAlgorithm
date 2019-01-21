#pragma once

#include <sofa/collisionAlgorithm/BaseElementContainer.h>

namespace sofa
{

namespace collisionAlgorithm
{

class BroadPhase : public core::collision::Pipeline
{
public:
    SOFA_ABSTRACT_CLASS(BroadPhase,core::collision::Pipeline);

    Data<defaulttype::Vector4> d_color;

    BroadPhase()
    : d_color(initData(&d_color, defaulttype::Vector4(1,0,1,1), "color", "Color of the collision model"))
    , l_elements(initLink("elements", "link to state")) {}

    virtual ~BroadPhase() {
        if (l_elements) l_elements->unsetBroadPhase(this);
    }

    void init( ) override {
        if (l_elements) l_elements->setBroadPhase(this);
    }


    virtual defaulttype::BoundingBox getBBox() const = 0;

    virtual bool selectElement(const defaulttype::Vector3 & P,std::set<unsigned> & eid, unsigned d = 0) const = 0;

    virtual void computeCollisionReset() override {}

    virtual void computeCollisionDetection() override {}

    virtual void computeCollisionResponse() override {}

    bool findDataLinkDest(BaseDataElmtContainer *& ptr, const std::string& path, const core::objectmodel::BaseLink* link)
    {
        core::objectmodel::BaseData* base = NULL;
        if (!this->getContext()->findDataLinkDest(base, path, link)) return false;
        ptr = dynamic_cast<BaseDataElmtContainer*>(base);
        return (ptr != NULL);
    }

    core::objectmodel::SingleLink<BroadPhase,BaseDataElmtContainer,BaseLink::FLAG_STOREPATH|BaseLink::FLAG_DATALINK> l_elements;

private:
    virtual void reset() override {}


    virtual void doCollisionReset() override {}

    virtual void doCollisionDetection(const sofa::helper::vector<core::CollisionModel*>& /*collisionModels*/) override {}

    virtual void doCollisionResponse() override {}

    virtual std::set< std::string > getResponseList() const override {
        std::set< std::string > res;
        return res;
    }

};


}

}
