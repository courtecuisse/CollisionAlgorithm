#pragma once

#include <sofa/collisionAlgorithm/BaseAlgorithm.h>
#include <sofa/collisionAlgorithm/BaseGeometry.h>

namespace sofa {

namespace collisionAlgorithm {

/*!
 * \brief The PointCloudBindingAlgorithm class
 * Component to bind points belonging to 2 different but similar Geometrie
 * Implements BaseAlgorithm
 */
class PointCloudBindingAlgorithm : public BaseAlgorithm
{
public:
    typedef sofa::defaulttype::Vector3 Vector3;
    SOFA_CLASS(PointCloudBindingAlgorithm, BaseAlgorithm);

    Data<double> d_maxDist;

    PointCloudBindingAlgorithm();

    void processAlgorithm(const BaseGeometry * geometry1, const BaseGeometry * g2, helper::vector< PairDetection > & output);

    static void bind(const std::vector<defaulttype::Vector3> & p1, const std::vector<defaulttype::Vector3> & p2, helper::vector<int> & bindId, helper::vector<int> & invBind, double maxDist);
};

}

}
