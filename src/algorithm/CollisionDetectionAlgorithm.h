#pragma once

#include <Collision.h>

namespace collisionAlgorithm {

class CollisionDetectionAlgorithm : public Collision {
public:

    Port<BaseGeometry,REQUIRED> p_from;
    Port<BaseGeometry,REQUIRED> p_dest;

    CollisionDetectionAlgorithm();

    void processAlgorithm();

private:

    PairProximity getClosestPoint(ConstraintElementPtr efrom);

};

}
