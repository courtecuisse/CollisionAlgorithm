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

    virtual void doDetection(const BaseGeometry * from, const BaseGeometry * dst, DetectionOutput & output) override;

    DetectionOutput::PairDetection findClosestPoint(const BaseElement::UPtr & itfrom, const BaseGeometry * geo);

    BaseProximity::SPtr findClosestPoint(BaseProximity::SPtr from, const BaseGeometry * geo);

private:
    BaseElementIterator::UPtr getDestIterator(const defaulttype::Vector3 & P, const BaseGeometry * geo);

};

}

}
