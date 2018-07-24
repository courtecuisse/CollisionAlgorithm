#pragma once

#include <Collision.h>
#include <animationLoop/AnimationLoop.h>
#include <core/Event.h>

namespace collisionAlgorithm {

class CollisionAnimationLoop : public AnimationLoop {
public:

    void step();

    void handleEvent(Event * );

};

}
