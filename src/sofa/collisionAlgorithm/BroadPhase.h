#pragma once

#include <sofa/core/collision/Pipeline.h>
#include <sofa/collisionAlgorithm/BaseGeometry.h>

namespace sofa {

namespace collisionAlgorithm {

class BroadPhase : public core::BehaviorModel
{
public:
    SOFA_ABSTRACT_CLASS(BroadPhase,core::BehaviorModel);

    Data<defaulttype::Vector4> d_color;
    core::objectmodel::SingleLink<BroadPhase,BaseGeometry,BaseLink::FLAG_STRONGLINK|BaseLink::FLAG_STOREPATH> l_geometry;

    BroadPhase()
    : d_color(initData(&d_color, defaulttype::Vector4(1,0,1,1), "color", "Color of the collision model"))
    , l_geometry(initLink("geometry", "link to state")) {}

    virtual ~BroadPhase() {
        if (l_geometry != NULL) l_geometry->setBroadPhase(NULL);
    }

    void init( ) override {
        if (l_geometry != NULL) l_geometry->setBroadPhase(this);
    }


    virtual void prepareDetection() = 0;

    virtual defaulttype::Vec3i getBoxSize() const = 0;

    virtual defaulttype::Vec3i getBoxCoord(const defaulttype::Vector3 & P) const = 0;

    virtual void getElementSet(unsigned cx,unsigned cy, unsigned cz, std::set<unsigned> & selectElements) const = 0;

private:

    virtual void updatePosition(SReal /*dt*/) {
        prepareDetection();
    }


};


}

}
