#pragma once

#include <sofa/collisionAlgorithm/BaseCollisionAlgorithm.h>
#include <sofa/collisionAlgorithm/BaseDecorator.h>

namespace sofa
{

namespace collisionAlgorithm
{

class CollisionDetectionAlgorithm : public BaseCollisionAlgorithm
{
public:
    SOFA_CLASS(CollisionDetectionAlgorithm, BaseCollisionAlgorithm);

    Data<double> d_minDist;
    Data<double> d_minAngle;
    Data<DetectionOutput> d_output;

    CollisionDetectionAlgorithm()
    : BaseCollisionAlgorithm()
    , d_minDist(initData(&d_minDist, std::numeric_limits<double>::max(), "dist", "this"))
    , d_minAngle(initData(&d_minAngle, -1.0, "angle","this"))
    , d_output(initData(&d_output, "output" , "this"))
    , l_from(initLink("from", "Link to from geometry"))
    , l_dest(initLink("dest", "Link to dest geometry"))
    {}

    virtual void computeCollisionReset() override;

    virtual void computeCollisionDetection() override;

private:
//    template<class ElementIterator>
    DetectionOutput::PairDetection findClosestPoint(BaseElement::Iterator &itfrom);

    core::objectmodel::SingleLink<CollisionDetectionAlgorithm,BaseGeometry,BaseLink::FLAG_STRONGLINK|BaseLink::FLAG_STOREPATH> l_from;
    core::objectmodel::SingleLink<CollisionDetectionAlgorithm,BaseGeometry,BaseLink::FLAG_STRONGLINK|BaseLink::FLAG_STOREPATH> l_dest;

    BaseElement::Iterator begin(const defaulttype::Vector3 & P) const;

};

}

}
