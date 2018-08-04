#pragma once

#include <Collision.h>
#include <decorator/AABBDecorator.h>

namespace collisionAlgorithm {

class CollisionDetectionAlgorithm : public Collision {
public:



    Port<BaseGeometry,IN> p_from;
    Port<BaseGeometry,IN> p_dest;

    CollisionDetectionAlgorithm();

    void processAlgorithm();

private:

    template<class ElementIterator>
    PairProximity getClosestPoint(ElementIterator geo);
};

}
