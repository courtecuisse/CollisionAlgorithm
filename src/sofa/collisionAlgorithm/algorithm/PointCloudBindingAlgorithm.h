#pragma once

#include <sofa/collisionAlgorithm/BaseGeometryAlgorithm.h>

namespace sofa
{

namespace collisionAlgorithm
{

class PointCloudBindingAlgorithm : public BaseGeometryAlgorithm
{
public:
    typedef sofa::defaulttype::Vector3 Vector3;
    SOFA_CLASS(PointCloudBindingAlgorithm, BaseGeometryAlgorithm);

    Data<double> d_maxDist;
    Data<DetectionOutput> d_output;

    PointCloudBindingAlgorithm();

    void computeCollisionReset() override;

    void computeCollisionDetection() override;

private:
    core::objectmodel::SingleLink<PointCloudBindingAlgorithm,BaseDataElmt,BaseLink::FLAG_STOREPATH|BaseLink::FLAG_DATALINK> l_from;
    core::objectmodel::SingleLink<PointCloudBindingAlgorithm,BaseDataElmt,BaseLink::FLAG_STOREPATH|BaseLink::FLAG_DATALINK> l_dest;
};

}

}
