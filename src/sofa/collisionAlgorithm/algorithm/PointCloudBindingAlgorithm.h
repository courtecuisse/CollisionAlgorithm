#pragma once

#include <sofa/collisionAlgorithm/BaseCollisionAlgorithm.h>

namespace sofa
{

namespace collisionAlgorithm
{

class PointCloudBindingAlgorithm : public BaseCollisionAlgorithm
{
public:
    typedef sofa::defaulttype::Vector3 Vector3;
    SOFA_CLASS(PointCloudBindingAlgorithm, BaseCollisionAlgorithm);

    Data<double> d_maxDist;
    Data<DetectionOutput> d_output;

    PointCloudBindingAlgorithm();

    void computeCollisionReset() override;

    void computeCollisionDetection() override;

private:
    core::objectmodel::SingleLink<PointCloudBindingAlgorithm,BaseGeometry,BaseLink::FLAG_STRONGLINK|BaseLink::FLAG_STOREPATH> l_from;
    core::objectmodel::SingleLink<PointCloudBindingAlgorithm,BaseGeometry,BaseLink::FLAG_STRONGLINK|BaseLink::FLAG_STOREPATH> l_dest;
};

}

}
