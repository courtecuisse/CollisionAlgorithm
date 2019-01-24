#pragma once

#include <sofa/core/collision/Pipeline.h>
#include <sofa/collisionAlgorithm/BaseGeometry.h>

namespace sofa {

namespace collisionAlgorithm {

class BroadPhase : public core::collision::Pipeline
{
public:
    SOFA_ABSTRACT_CLASS(BroadPhase,core::collision::Pipeline);

    Data<defaulttype::Vector4> d_color;

    BroadPhase()
    : d_color(initData(&d_color, defaulttype::Vector4(1,0,1,1), "color", "Color of the collision model"))
    , l_geometry(initLink("geometry", "link to state")) {}

    virtual ~BroadPhase() {
        if (l_geometry != NULL) l_geometry->unsetBroadPhase(this);
    }

    void init( ) override {
        if (l_geometry != NULL) l_geometry->setBroadPhase(this);
    }


    virtual defaulttype::BoundingBox getBBox() const = 0;

    virtual bool selectElement(const defaulttype::Vector3 & P,std::set<unsigned> & eid, unsigned d = 0) const = 0;

    virtual void computeCollisionReset() override {}

    virtual void computeCollisionDetection() override {}

    virtual void computeCollisionResponse() override {}

    core::objectmodel::SingleLink<BroadPhase,BaseGeometry,BaseLink::FLAG_STRONGLINK|BaseLink::FLAG_STOREPATH> l_geometry;

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
