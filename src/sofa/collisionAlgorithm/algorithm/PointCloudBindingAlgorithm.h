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
    Data<DataElementIterator> d_from;
    Data<DataElementIterator> d_dest;

    PointCloudBindingAlgorithm();

    void computeCollisionReset() override;

    void computeCollisionDetection() override;

//private:
//    core::objectmodel::SingleLink<PointCloudBindingAlgorithm,DataIterator,BaseLink::FLAG_STRONGLINK|BaseLink::FLAG_STOREPATH|BaseLink::FLAG_DATALINK> l_from;
//    core::objectmodel::SingleLink<PointCloudBindingAlgorithm,DataIterator,BaseLink::FLAG_STRONGLINK|BaseLink::FLAG_STOREPATH|BaseLink::FLAG_DATALINK> l_dest;
};

}

}
