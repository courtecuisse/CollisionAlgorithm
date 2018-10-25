#pragma once

#include <sofa/collisionAlgorithm/BaseCollisionAlgorithm.h>
#include <sofa/collisionAlgorithm/decorator/AABBDecorator.h>

namespace sofa {

namespace collisionAlgorithm {

class CollisionDetectionAlgorithm : public BaseCollisionAlgorithm {
public:

    DataLink<BaseGeometry> d_from;
    DataLink<BaseGeometry> d_dest;
    Data<double> d_minDist;
    Data<double> d_minAngle;

    CollisionDetectionAlgorithm()
    : d_from(initData(&d_from,"from", "this"))
    , d_dest(initData(&d_dest,"dest", "this"))
    , d_minDist(initData(&d_minDist, std::numeric_limits<double>::max(), "dist", "this"))
    , d_minAngle(initData(&d_minAngle, -1.0, "angle","this"))
    {}

    void processAlgorithm();

    void getState(std::set<sofa::core::behavior::MechanicalState<defaulttype::Vec3dTypes>* > & list_state) {
        list_state.insert(d_from->getState());
        list_state.insert(d_dest->getState());
    }

private:
    template<class ElementIterator>
    PairProximity getClosestPoint(ElementIterator geo);

};

}

}
