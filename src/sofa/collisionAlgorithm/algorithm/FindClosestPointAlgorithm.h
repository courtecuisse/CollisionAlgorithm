#pragma once

#include <sofa/collisionAlgorithm/BaseGeometryAlgorithm.h>
#include <sofa/collisionAlgorithm/BroadPhase.h>
#include <sofa/collisionAlgorithm/BaseGeometry.h>

namespace sofa
{

namespace collisionAlgorithm
{

class FindClosestPointAlgorithm : public BaseGeometryAlgorithm
{
public:
    SOFA_CLASS(FindClosestPointAlgorithm, BaseGeometryAlgorithm);

    Data<DetectionOutput> d_output;

    FindClosestPointAlgorithm()
    : d_output(initData(&d_output, "output" , "this"))
    , l_from(initLink("from", "link to from geometry elments"))
    , l_dest(initLink("dest", "link to dest geometry elments"))
    {}

    virtual void computeCollisionReset() override;

    virtual void computeCollisionDetection() override;

private:
//    template<class ElementIterator>

    core::objectmodel::SingleLink<FindClosestPointAlgorithm,BaseGeometry,BaseLink::FLAG_STRONGLINK|BaseLink::FLAG_STOREPATH> l_from;
    core::objectmodel::SingleLink<FindClosestPointAlgorithm,BaseGeometry,BaseLink::FLAG_STRONGLINK|BaseLink::FLAG_STOREPATH> l_dest;

    BaseElementIterator::UPtr getDestIterator(const defaulttype::Vector3 & P);

    DetectionOutput::PairDetection findClosestPoint(const BaseElement::UPtr & itfrom);
};

}

}
