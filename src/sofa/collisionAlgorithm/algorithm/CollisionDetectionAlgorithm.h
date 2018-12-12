#pragma once

#include <sofa/collisionAlgorithm/BaseCollisionAlgorithm.h>
#include <sofa/collisionAlgorithm/decorator/AABBDecorator.h>

namespace sofa {

namespace collisionAlgorithm {

class CollisionDetectionAlgorithm : public BaseCollisionAlgorithm {
public:
    SOFA_CLASS(CollisionDetectionAlgorithm, BaseCollisionAlgorithm);

    Data<double> d_minDist;
    Data<double> d_minAngle;

    CollisionDetectionAlgorithm()
    : d_minDist(initData(&d_minDist, std::numeric_limits<double>::max(), "dist", "this"))
    , d_minAngle(initData(&d_minAngle, -1.0, "angle","this"))
    , l_from(initLink("from", "Link to from geometry"))
    , l_dest(initLink("dest", "Link to dest geometry"))
    {}

    void processAlgorithm() override;

    void getState(std::set<sofa::core::behavior::MechanicalState<defaulttype::Vec3dTypes>* > & list_state) override
    {
        list_state.insert(l_from->getState());
        list_state.insert(l_dest->getState());
    }

private:
    template<class ElementIterator>
    PairProximity getClosestPoint(ElementIterator geo);


    core::objectmodel::SingleLink<CollisionDetectionAlgorithm,BaseGeometry,BaseLink::FLAG_STRONGLINK|BaseLink::FLAG_STOREPATH> l_from;
    core::objectmodel::SingleLink<CollisionDetectionAlgorithm,BaseGeometry,BaseLink::FLAG_STRONGLINK|BaseLink::FLAG_STOREPATH> l_dest;
};

}

}
