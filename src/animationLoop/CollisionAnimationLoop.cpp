#include <animationLoop/CollisionAnimationLoop.h>
#include <core/Timer.h>
#include <core/Factory.h>
#include <MechanicalVisitor.h>
#include <topology/Topology.h>
#include <state/State.h>
#include <Collision.h>

namespace collisionAlgorithm {

void CollisionAnimationLoop::step() {
    Timer::beginStep("Collision");
    class CollisionVisitor : public Visitor {
    public:
        CollisionVisitor() : Visitor() {}

        VISITOR_ACTION processObject(BaseObject * c) {
            if (Collision * i = dynamic_cast<Collision*>(c)) i->computeCollisionDetection();
            return VISITOR_CONTINUE;
        }
    };
    CollisionVisitor().execute(p_object);
    Timer::endStep("Collision");


    Timer::beginStep("Integration");
    class IntegrationVisitor : public Visitor {
    public:
        IntegrationVisitor(double dt) : Visitor() {
            m_dt = dt;
        }

        VISITOR_ACTION processObject(BaseObject * c) {
            if (Integrator * i = dynamic_cast<Integrator*>(c)) i->step(m_dt);
            return VISITOR_CONTINUE;
        }

        double m_dt;
    };
    IntegrationVisitor(d_step.getValue()).execute(p_object);
    Timer::endStep("Integration");

    Timer::beginStep("Mapping");
    MappingApplyVisitor().execute(p_object);
    Timer::endStep("Mapping");
}

void CollisionAnimationLoop::handleEvent(Event * e) {
    if (dynamic_cast<AnimateBeginEvent *>(e)) step();
}

DECLARE_CLASS(CollisionAnimationLoop)

}
