#pragma once

#include <sofa/collisionAlgorithm/BaseAlgorithm.h>
#include <sofa/collisionAlgorithm/BroadPhase.h>
#include <sofa/collisionAlgorithm/BaseGeometry.h>

namespace sofa
{

namespace collisionAlgorithm
{

class FindClosestPointAlgorithm : public BaseAlgorithm
{
public:
    SOFA_ABSTRACT_CLASS(FindClosestPointAlgorithm, BaseAlgorithm);

    FindClosestPointAlgorithm () ;
    void processAlgorithm(const BaseGeometry * geometry1, const BaseGeometry * g2, helper::vector< PairDetection > & output);

    BaseProximity::SPtr findClosestPoint(BaseProximity::SPtr from, const BaseGeometry * geo);


    core::objectmodel::SingleLink<FindClosestPointAlgorithm,BaseGeometry,BaseLink::FLAG_STRONGLINK|BaseLink::FLAG_STOREPATH> l_from;
    core::objectmodel::SingleLink<FindClosestPointAlgorithm,BaseGeometry,BaseLink::FLAG_STRONGLINK|BaseLink::FLAG_STOREPATH> l_dest;

    Data<DetectionOutput> d_output;
protected:

    void doDetection();

    BaseElementIterator::UPtr getDestIterator(const defaulttype::Vector3 & P, const BaseGeometry * geo);

    void fillElementSet(const BroadPhase * decorator, defaulttype::Vec3i cbox, std::set<unsigned> & selectElements, int d) const;

    BaseProximity::SPtr findClosestPoint(BaseProximity::SPtr pfrom, BaseElementIterator::UPtr itdest);

    PairDetection findClosestPoint(const BaseElement::UPtr & itfrom, const BaseGeometry * geo);

};


}

}
