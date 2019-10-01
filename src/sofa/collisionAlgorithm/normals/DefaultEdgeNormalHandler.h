#pragma once

#include <sofa/collisionAlgorithm/geometry/EdgeGeometry.h>

namespace sofa {

namespace collisionAlgorithm {

template<class DataTypes>
class DefaultEdgeNormalHandler : public EdgeGeometry<DataTypes>::EdgeNormalHandler {
public:
    typedef typename DataTypes::VecCoord VecCoord;
    typedef core::objectmodel::Data< VecCoord >        DataVecCoord;

    SOFA_CLASS(SOFA_TEMPLATE(DefaultEdgeNormalHandler,DataTypes), typename EdgeGeometry<DataTypes>::EdgeNormalHandler);

    defaulttype::Vector3 computeNormal(const EdgeProximity & data) const override {
        const helper::ReadAccessor<DataVecCoord> & pos = this->l_geometry->getState()->read(core::VecCoordId::position());
        return (pos[data.m_p1] - pos[data.m_p0]).normalized();
    }
};

}

}
