#pragma once

#include <sofa/collisionAlgorithm/BaseGeometryAlgorithm.h>
#include <sofa/collisionAlgorithm/BaseGeometry.h>

namespace sofa {

namespace collisionAlgorithm {

class PointCloudBindingAlgorithm : public BaseGeometryAlgorithm
{
public:
    typedef sofa::defaulttype::Vector3 Vector3;
    SOFA_CLASS(PointCloudBindingAlgorithm, BaseGeometryAlgorithm);

    Data<double> d_maxDist;

    PointCloudBindingAlgorithm();

    void doDetection(const BaseGeometry * from, const BaseGeometry * dst, DetectionOutput & output) override;
};

}

}
