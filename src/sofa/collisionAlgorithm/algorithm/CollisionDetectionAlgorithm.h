#pragma once

#include <sofa/collisionAlgorithm/BaseCollisionAlgorithm.h>
#include <sofa/collisionAlgorithm/decorator/AABBDecorator.h>

namespace sofa {

namespace collisionAlgorithm {

class CollisionDetectionAlgorithm : public BaseCollisionAlgorithm {
public:

    DataLink<BaseGeometry> d_from;
    DataLink<BaseGeometry> d_dest;

    CollisionDetectionAlgorithm()
    : d_from(initData(&d_from,"from", "this"))
    , d_dest(initData(&d_dest,"dest", "this")) {}

    void processAlgorithm();

private:
    template<class ElementIterator>
    PairProximity getClosestPoint(ElementIterator geo);

};

}

}
