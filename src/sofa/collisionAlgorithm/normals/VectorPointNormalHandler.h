#pragma once

#include <sofa/collisionAlgorithm/geometry/PointGeometry.h>

namespace sofa {

namespace collisionAlgorithm {

template<class DataTypes>
class VectorPointNormalHandler : public TBaseNormalHandler<DataTypes,PointProximity> {
public:
    SOFA_CLASS(SOFA_TEMPLATE(VectorPointNormalHandler,DataTypes), SOFA_TEMPLATE2(TBaseNormalHandler,DataTypes,PointProximity));

    Data<helper::vector<defaulttype::Vector3>> d_normals;

    VectorPointNormalHandler()
    : d_normals(initData(&d_normals, "normals", "Vector of normals")) {}

    virtual defaulttype::Vector3 computeNormal(const PointProximity & data) const {
        if(data.m_eid < d_normals.getValue().size()) return d_normals.getValue()[data.m_eid];
        if(d_normals.getValue().size()==1) return d_normals.getValue()[0];
        return defaulttype::Vector3();
    }
};

}

}
