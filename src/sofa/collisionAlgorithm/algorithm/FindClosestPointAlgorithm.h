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

    CollisionDetectionAlgorithm()
    : d_output(initData(&d_output, "output" , "this"))
    , l_from(initLink("from", "link to from geometry elments"))
    , l_dest(initLink("dest", "link to dest geometry elments"))
    {}

    virtual void computeCollisionReset() override;

    virtual void computeCollisionDetection() override;

private:
//    template<class ElementIterator>

    core::objectmodel::SingleLink<CollisionDetectionAlgorithm,BaseDataElmt,BaseLink::FLAG_STOREPATH|BaseLink::FLAG_DATALINK> l_from;
    core::objectmodel::SingleLink<CollisionDetectionAlgorithm,BaseDataElmt,BaseLink::FLAG_STOREPATH|BaseLink::FLAG_DATALINK> l_dest;

    BaseElementIterator::UPtr getDestIterator(const defaulttype::Vector3 & P);

    DetectionOutput::PairDetection findClosestPoint(const BaseElementIterator::UPtr & itfrom);
};

}

}
