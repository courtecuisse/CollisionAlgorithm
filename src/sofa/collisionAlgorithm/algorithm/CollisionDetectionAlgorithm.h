#pragma once

#include <sofa/collisionAlgorithm/BaseGeometryAlgorithm.h>
#include <sofa/collisionAlgorithm/BroadPhase.h>

namespace sofa
{

namespace collisionAlgorithm
{

class CollisionDetectionAlgorithm : public BaseGeometryAlgorithm
{
public:
    SOFA_CLASS(CollisionDetectionAlgorithm, BaseGeometryAlgorithm);

    Data<DetectionOutput> d_output;
    Data<DataElementIterator> d_from;
    Data<DataElementIterator> d_dest;

    CollisionDetectionAlgorithm()
    : d_output(initData(&d_output, "output" , "this"))
    , d_from(initData(&d_from, "from" , "this"))
    , d_dest(initData(&d_dest, "dest" , "this"))
    {}

    virtual void computeCollisionReset() override;

    virtual void computeCollisionDetection() override;


private:
//    template<class ElementIterator>

//    core::objectmodel::SingleLink<CollisionDetectionAlgorithm,DataIterator,BaseLink::FLAG_STRONGLINK|BaseLink::FLAG_STOREPATH|BaseLink::FLAG_DATALINK> l_from;
//    core::objectmodel::SingleLink<CollisionDetectionAlgorithm,DataIterator,BaseLink::FLAG_STRONGLINK|BaseLink::FLAG_STOREPATH|BaseLink::FLAG_DATALINK> l_dest;

    BaseElementIterator::UPtr getDestIterator(const defaulttype::Vector3 & P);

    DetectionOutput::PairDetection findClosestPoint(const BaseElementIterator::UPtr & itfrom);
};

}

}
