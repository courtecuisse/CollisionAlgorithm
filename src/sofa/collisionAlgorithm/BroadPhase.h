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
    Data<DataElementIterator> d_elements;

    BroadPhase()
    : d_color(initData(&d_color, defaulttype::Vector4(1,0,1,1), "color", "Color of the collision model"))
    , d_elements(initData(&d_elements, "geometry", "link to state")) {}

    virtual ~BroadPhase() {
        d_elements.getValue().unsetBroadPhase(this);
    }

    virtual defaulttype::BoundingBox getBBox() const = 0;

    virtual bool selectElement(const defaulttype::Vector3 & P,std::set<unsigned> & eid, unsigned d = 0) const = 0;

    void init( ) override {
        if (d_elements.getValue().end() != NULL) {
            d_elements.getValue().unsetBroadPhase(this);
        }
    }

    virtual void prepareDetection() = 0;

//    core::objectmodel::SingleLink<BroadPhase,DataIterator,BaseLink::FLAG_STRONGLINK|BaseLink::FLAG_STOREPATH|BaseLink::FLAG_DATALINK> l_geometry;
};


}

}
