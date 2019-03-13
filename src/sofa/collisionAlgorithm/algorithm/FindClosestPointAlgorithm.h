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

    void processAlgorithm(BaseElementIterator::UPtr it, const BaseGeometry * g2, helper::vector< PairDetection > & output);

    PairDetection findClosestPoint(const BaseElement::UPtr & itfrom, const BaseGeometry * geo);

    BaseProximity::SPtr findClosestPoint(BaseProximity::SPtr from, const BaseGeometry * geo);

    BaseProximity::SPtr findClosestPoint(BaseProximity::SPtr pfrom, BaseElementIterator::UPtr itdest);

protected:
    BaseElementIterator::UPtr getDestIterator(const defaulttype::Vector3 & P, const BaseGeometry * geo);

    void fillElementSet(const BroadPhase * decorator, defaulttype::Vec3i cbox, std::set<unsigned> & selectElements, int d) const;

};


}

}
