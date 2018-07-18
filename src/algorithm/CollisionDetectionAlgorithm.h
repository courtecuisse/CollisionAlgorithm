#pragma once

#include <Collision.h>

namespace collisionAlgorithm {

class CollisionDetectionAlgorithm : public Collision {
public:

    Port<BaseGeometry,REQUIRED> p_from;
    Port<BaseGeometry,REQUIRED> p_dest;

    CollisionDetectionAlgorithm();

    void processAlgorithm();

//    virtual void handleEvent(Event * e) {
//        p_from->handleEvent(e);
//        p_dest->handleEvent(e);
//        Collision::handleEvent(e);
//    }

private:

    PairProximity getClosestPoint(ConstraintElementPtr efrom);

};

}
