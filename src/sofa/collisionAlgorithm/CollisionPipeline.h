#pragma once

//#include <sofa/collisionAlgorithm/BaseOperation.h>
#include <sofa/simulation/CollisionBeginEvent.h>
#include <sofa/collisionAlgorithm/BaseElement.h>
#include <sofa/collisionAlgorithm/BaseProximity.h>
#include <sofa/core/BehaviorModel.h>
#include <sofa/core/visual/VisualParams.h>
#include <sofa/core/objectmodel/DataCallback.h>
#include <sofa/simulation/Visitor.h>
#include <sofa/gl/gl.h>
#include <sofa/simulation/AnimateBeginEvent.h>
#include <sofa/core/topology/BaseMeshTopology.h>
#include <sofa/collisionAlgorithm/BaseElement.h>
#include <sofa/simulation/Node.h>

namespace sofa ::collisionAlgorithm {

class CollisionComponent : public core::objectmodel::BaseObject {
public:

    virtual void update() = 0;

};

class CollisionPipeline : public core::objectmodel::BaseObject {
public:

    SOFA_ABSTRACT_CLASS(CollisionPipeline,core::objectmodel::BaseObject);

    class UpdateCollisionVisitor : public simulation::Visitor {
    public:
        UpdateCollisionVisitor() : Visitor(sofa::core::ExecParams::defaultInstance()) {}

        Visitor::Result processNodeTopDown(simulation::Node* node) {
            for_each(this, node, node->object, &UpdateCollisionVisitor::processObject);
            return Visitor::RESULT_CONTINUE;
        }

        void processObject(simulation::Node*, core::objectmodel::BaseObject* obj) {
            if (CollisionComponent * component = dynamic_cast<CollisionComponent *>(obj)) {
                component->update();
            }
        }

        virtual const char* getClassName() const {return "UpdateCollisionVisitor";}
    };

    CollisionPipeline() {
        this->f_listening.setValue(true);
    }

    void handleEvent(sofa::core::objectmodel::Event *event) {
        if (! dynamic_cast<sofa::simulation::AnimateBeginEvent*>(event)) return;

        UpdateCollisionVisitor v;
        v.execute(this->getContext());
    }

};

}

