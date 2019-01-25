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
    Data<DetectionOutput> d_output;

    PointCloudBindingAlgorithm();

    void doDetection() override;

private:
    core::objectmodel::SingleLink<PointCloudBindingAlgorithm,BaseGeometry,BaseLink::FLAG_STRONGLINK|BaseLink::FLAG_STOREPATH> l_from;
    core::objectmodel::SingleLink<PointCloudBindingAlgorithm,BaseGeometry,BaseLink::FLAG_STRONGLINK|BaseLink::FLAG_STOREPATH> l_dest;
};

}

}
