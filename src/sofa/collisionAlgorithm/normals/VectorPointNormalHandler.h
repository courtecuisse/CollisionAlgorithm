#pragma once

#include <sofa/collisionAlgorithm/BaseNormalHandler.h>

namespace sofa {

namespace collisionAlgorithm {

template<class GEOMETRY>
class VectorPointNormalHandler : public TBaseNormalHandler<GEOMETRY> {
public:
    typedef typename GEOMETRY::VecCoord VecCoord;
    typedef typename GEOMETRY::DataVecCoord DataVecCoord;
    typedef typename GEOMETRY::PROXIMITYDATA PROXIMITYDATA;
    typedef TBaseNormalHandler<GEOMETRY> Inherit;

    SOFA_CLASS(SOFA_TEMPLATE(VectorPointNormalHandler,GEOMETRY), Inherit);

    Data<helper::vector<defaulttype::Vector3>> d_normals;

    VectorPointNormalHandler()
    : d_normals(initData(&d_normals, "normals", "Vector of normals")) {}

    virtual defaulttype::Vector3 computeNormal(const PROXIMITYDATA & data) const {
        if(data.m_eid < d_normals.getValue().size()) return d_normals.getValue()[data.m_eid];
        if(d_normals.getValue().size()==1) return d_normals.getValue()[0];
        return defaulttype::Vector3();
    }

    virtual void updateNormals() override {}
};

}

}
