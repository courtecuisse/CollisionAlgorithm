#pragma once

#include <sofa/collisionAlgorithm/geometry/PointGeometry.h>

namespace sofa {

namespace collisionAlgorithm {

template<class DataTypes>
class VectorPointNormalHandler : public PointGeometry<DataTypes>::PointNormalHandler {
public:
    SOFA_CLASS(SOFA_TEMPLATE(VectorPointNormalHandler,DataTypes), typename PointGeometry<DataTypes>::PointNormalHandler);

    Data<helper::vector<defaulttype::Vector3>> d_normals;

    VectorPointNormalHandler()
    : d_normals(initData(&d_normals, "normals", "Vector of normals")) {}

    virtual defaulttype::Vector3 computeNormal(const PointProximity & data) const {
        if(data.m_eid < d_normals.getValue().size()) return d_normals.getValue()[data.m_eid];
        return defaulttype::Vector3();
    }
};

}

}
