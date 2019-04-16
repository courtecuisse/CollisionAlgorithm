#pragma once

#include <sofa/collisionAlgorithm/BaseAlgorithm.h>
#include <sofa/collisionAlgorithm/BroadPhase.h>
#include <sofa/collisionAlgorithm/BaseGeometry.h>

#include <sofa/collisionAlgorithm/BaseDistanceMeasure.h>

namespace sofa
{

namespace collisionAlgorithm
{

class FindClosestPointAlgorithm : public BaseAlgorithm
{
public:
    SOFA_ABSTRACT_CLASS(FindClosestPointAlgorithm, BaseAlgorithm);

    FindClosestPointAlgorithm () ;
    void init () ;

    core::objectmodel::SingleLink<
        FindClosestPointAlgorithm,
        BaseGeometry,
        BaseLink::FLAG_STRONGLINK|BaseLink::FLAG_STOREPATH> l_from;
    core::objectmodel::SingleLink<
        FindClosestPointAlgorithm,
        BaseGeometry,
        BaseLink::FLAG_STRONGLINK|BaseLink::FLAG_STOREPATH> l_dest;

    core::objectmodel::SingleLink<
        FindClosestPointAlgorithm,
        BaseDistanceMeasure,
        BaseLink::FLAG_STRONGLINK|BaseLink::FLAG_STOREPATH> l_distance_measure;

    Data<DetectionOutput> d_output;


protected:

    BaseDistanceMeasure* m_distance_measure ;

    void doDetection();

    BaseElementIterator::UPtr getDestIterator(const defaulttype::Vector3 & P, BaseGeometry *geo);

    void fillElementSet(const BroadPhase * decorator, defaulttype::Vec3i cbox, std::set<unsigned> & selectElements, int d) const;

public:
    BaseProximity::SPtr findClosestPoint(BaseProximity::SPtr pfrom, BaseElementIterator::UPtr itdest);

    PairDetection findClosestPoint(const BaseElement::UPtr & itfrom, BaseGeometry *geo);

    BaseProximity::SPtr findClosestPoint(BaseProximity::SPtr from, BaseGeometry *geo);

};


}

}
