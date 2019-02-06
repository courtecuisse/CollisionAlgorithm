#pragma once

#include <sofa/collisionAlgorithm/BaseGeometryAlgorithm.h>
#include <sofa/collisionAlgorithm/BroadPhase.h>
#include <sofa/collisionAlgorithm/BaseGeometry.h>

namespace sofa
{

namespace collisionAlgorithm
{

class BaseClosestPointAlgorithm : public BaseGeometryAlgorithm
{
public:
    SOFA_ABSTRACT_CLASS(BaseClosestPointAlgorithm, BaseGeometryAlgorithm);

    DetectionOutput::PairDetection findClosestPoint(const BaseElement::UPtr & itfrom, const BaseGeometry * geo);

    BaseProximity::SPtr findClosestPoint(BaseProximity::SPtr from, const BaseGeometry * geo);

    BaseProximity::SPtr findClosestPoint(BaseProximity::SPtr pfrom, BaseElementIterator::UPtr itdest);

private:
    BaseElementIterator::UPtr getDestIterator(const defaulttype::Vector3 & P, const BaseGeometry * geo);

};


class FindClosestPointAlgorithm : public BaseClosestPointAlgorithm
{
public:
    SOFA_CLASS(FindClosestPointAlgorithm, BaseClosestPointAlgorithm);

    core::objectmodel::SingleLink<FindClosestPointAlgorithm,BaseGeometry,BaseLink::FLAG_STRONGLINK|BaseLink::FLAG_STOREPATH> l_from;
    core::objectmodel::SingleLink<FindClosestPointAlgorithm,BaseGeometry,BaseLink::FLAG_STRONGLINK|BaseLink::FLAG_STOREPATH> l_dest;
    Data<DetectionOutput> d_output;

    void doDetection();

    FindClosestPointAlgorithm()
    : l_from(initLink("from", "link to from geometry"))
    , l_dest(initLink("dest", "link to dest geometry"))
    , d_output(initData(&d_output,"output", "output of the collision detection")){}

};


}

}
