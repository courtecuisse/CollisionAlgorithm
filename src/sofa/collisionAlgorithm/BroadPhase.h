#pragma once

#include <sofa/collisionAlgorithm/BaseGeometry.h>
#include <sofa/collisionAlgorithm/BaseElement.h>
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
    , l_elements(initLink("elements", "link to state")) {}

    virtual ~BroadPhase() {
        if (l_elements) l_elements->unsetBroadPhase(this);
    }

    virtual defaulttype::BoundingBox getBBox() const = 0;

    virtual bool selectElement(const defaulttype::Vector3 & P,std::set<unsigned> & eid, unsigned d = 0) const = 0;

    void init( ) override {
        if (l_elements) l_elements->setBroadPhase(this);
    }

    virtual void prepareDetection() = 0;

    bool findDataLinkDest(BaseDataElmt *& ptr, const std::string& path, const core::objectmodel::BaseLink* link)
    {
        core::objectmodel::BaseData* base = NULL;
        if (!this->getContext()->findDataLinkDest(base, path, link)) return false;
        ptr = dynamic_cast<BaseDataElmt*>(base);
        return (ptr != NULL);
    }

    core::objectmodel::SingleLink<BroadPhase,BaseDataElmt,BaseLink::FLAG_STOREPATH|BaseLink::FLAG_DATALINK> l_elements;
};


}

}
