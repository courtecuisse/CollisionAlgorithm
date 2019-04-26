#pragma once

#include <sofa/collisionAlgorithm/BaseAlgorithm.h>
#include <sofa/collisionAlgorithm/BroadPhase.h>
#include <sofa/collisionAlgorithm/BaseGeometry.h>

#include <sofa/collisionAlgorithm/data/DataDistanceMeasure.h>

namespace sofa
{

namespace collisionAlgorithm
{

class FindClosestPointAlgorithm : public BaseAlgorithm
{
public:
    SOFA_CLASS(FindClosestPointAlgorithm, BaseAlgorithm);

    Data<unsigned> d_iterations;

    core::objectmodel::SingleLink<FindClosestPointAlgorithm,BaseGeometry,BaseLink::FLAG_STOREPATH|BaseLink::FLAG_STRONGLINK> l_from;
    core::objectmodel::SingleLink<FindClosestPointAlgorithm,BaseGeometry,BaseLink::FLAG_STOREPATH|BaseLink::FLAG_STRONGLINK> l_dest;

    Data<DistanceMeasure> d_distance_measure;

    Data<DetectionOutput> d_output;

    FindClosestPointAlgorithm();

protected:

    void doDetection();

    BaseElementIterator::UPtr getDestIterator(const defaulttype::Vector3 & P, BaseGeometry *geo);

    void fillElementSet(const BroadPhase * decorator, defaulttype::Vec3i cbox, std::set<unsigned> & selectElements, int d) const;

public:
    PairDetection dofindClosestPoint(const BaseElementIterator *elfrom, BaseElementIterator::UPtr itdest, BaseGeometry * geo);

    PairDetection findClosestPoint(const BaseElementIterator *elfrom, BaseGeometry *geo);

};


}

}
