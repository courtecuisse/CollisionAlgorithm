#pragma once

#include <BaseGeometry.h>
#include <Collision.h>

namespace collisionAlgorithm {

class CollisionForceField :  public ForceField {
public:

    Data<double> d_stiffness;

    PortOut<Collision,REQUIRED> p_collision;

    CollisionForceField();

    void addForce(TVecId f);

    void addToMatrix(BaseMatrix *M);

    void draw(DisplayFlag);

};

}
