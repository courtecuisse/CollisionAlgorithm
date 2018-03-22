#pragma once

#include <Collision.h>

namespace graFE {

class CollisionDetectionAlgorithm : public Collision {
public:

    PortOut<BaseGeometry,REQUIRED> p_from;
    PortOut<BaseGeometry,REQUIRED> p_dest;

    CollisionDetectionAlgorithm();

    void processAlgorithm();

private:

    PairProximity getClosestPoint(ConstraintElementPtr efrom);

};

}
