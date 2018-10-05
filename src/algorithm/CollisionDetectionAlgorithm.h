#pragma once

#include <Collision.h>
#include <decorator/AABBDecorator.h>

namespace sofa {

namespace collisionAlgorithm {

class CollisionDetectionAlgorithm : public Collision {
public:

    DataLink<BaseGeometry> d_from;
    DataLink<BaseGeometry> d_dest;

    CollisionDetectionAlgorithm()
    : d_from("from", this)
    , d_dest("dest", this) {}

    void processAlgorithm();

private:
    template<class ElementIterator>
    PairProximity getClosestPoint(ElementIterator geo);

};

}

}
