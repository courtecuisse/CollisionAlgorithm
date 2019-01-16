#pragma once

#include <sofa/collisionAlgorithm/BaseGeometry.h>
#include <sofa/core/visual/VisualParams.h>
#include <sofa/simulation/AnimateBeginEvent.h>
#include <sofa/core/objectmodel/BaseObject.h>
#include <sofa/core/behavior/MechanicalState.h>
#include <sofa/core/BehaviorModel.h>
#include <sofa/core/collision/Pipeline.h>
#include <sofa/collisionAlgorithm/BaseProximity.h>
#include <sofa/core/topology/BaseMeshTopology.h>
#include <memory>
#include <map>
#include <vector>
#include <qopengl.h>

namespace sofa
{

namespace collisionAlgorithm
{

class BroadPhase : public core::objectmodel::BaseObject
{
public:
    SOFA_ABSTRACT_CLASS(BroadPhase,core::objectmodel::BaseObject);

    Data<defaulttype::Vector4> d_color;

    BroadPhase()
    : d_color(initData(&d_color, defaulttype::Vector4(1,0,1,1), "color", "Color of the collision model"))
    , l_geometry(initLink("geometry", "link to state")) {
        l_geometry.setPath("@.");
    }

    virtual ~BroadPhase() {
        l_geometry->unsetDecorator(this);
    }

    virtual defaulttype::BoundingBox getBBox() const = 0;

    virtual bool selectElement(const defaulttype::Vector3 & P,std::set<unsigned> & eid, unsigned d = 0) const = 0;

    void init( ) override {
        if (l_geometry != NULL) l_geometry->setDecorator(this);
    }

    virtual void prepareDetection() = 0;

    core::objectmodel::SingleLink<BroadPhase,BaseGeometry,BaseLink::FLAG_STRONGLINK|BaseLink::FLAG_STOREPATH|BaseLink::FLAG_DOUBLELINK> l_geometry;
};


}

}
