#pragma once

#include <sofa/core/topology/BaseMeshTopology.h>
#include <sofa/collisionAlgorithm/BaseGeometry.h>

namespace sofa
{

namespace collisionAlgorithm
{

class PointElement;

class PointGeometry : public BaseGeometry
{
    friend class PointElement;
    friend class PointProximity;
public:
    SOFA_CLASS(PointGeometry,BaseGeometry);

    BaseProximity::SPtr createProximity(const PointElement * elmt) const;

    virtual void prepareDetection() override;

    virtual void init() override;

};

}

}
