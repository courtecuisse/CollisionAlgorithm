#pragma once

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

class BaseDecorator : public core::collision::Pipeline
{
public:
    SOFA_ABSTRACT_CLASS(BaseDecorator,core::collision::Pipeline);

    Data<defaulttype::Vector4> d_color;

    BaseDecorator()
    : d_color(initData(&d_color, defaulttype::Vector4(1,0,1,1), "color", "Color of the collision model"))
    {}

    virtual BaseElement::Iterator begin(const defaulttype::Vector3 & P) const = 0;

    virtual const BaseDecorator * end() const {
        return this;
    }

    virtual sofa::core::behavior::BaseMechanicalState * getState() const = 0;

    void bwdInit( ) override {
        prepareDetection();
    }

    virtual void reset() override {}

    virtual void computeCollisionReset() override {
        prepareDetection();
    }

    virtual void computeCollisionResponse() override {}

    void computeCollisionDetection() override {}

protected:
    virtual void doCollisionReset() override {}

    virtual void doCollisionDetection(const sofa::helper::vector<core::CollisionModel*>& /*collisionModels*/) override {}

    virtual void doCollisionResponse() override {}

    virtual std::set< std::string > getResponseList() const override {
        std::set< std::string > res;
        return res;
    }

    virtual void prepareDetection() {}
};


}

}
