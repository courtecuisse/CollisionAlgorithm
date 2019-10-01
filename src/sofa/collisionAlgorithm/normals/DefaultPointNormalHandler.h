#pragma once

#include <sofa/collisionAlgorithm/geometry/PointGeometry.h>

namespace sofa {

namespace collisionAlgorithm {

template<class DataTypes>
class DefaultPointNormalHandler : public PointGeometry<DataTypes>::PointNormalHandler {
public:
    SOFA_CLASS(SOFA_TEMPLATE(DefaultPointNormalHandler,DataTypes), typename PointGeometry<DataTypes>::PointNormalHandler);

    defaulttype::Vector3 computeNormal(const PointProximity & /*data*/) const override {
        defaulttype::Vector3();
    }
};

}

}
