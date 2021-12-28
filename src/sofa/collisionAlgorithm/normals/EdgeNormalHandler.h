//#pragma once

//#include <sofa/collisionAlgorithm/BaseNormalHandler.h>

//namespace sofa {

//namespace collisionAlgorithm {

//template<class GEOMETRY>
//class EdgeNormalHandler : public TBaseNormalHandler<GEOMETRY> {
//public:
//    typedef typename GEOMETRY::VecCoord VecCoord;
//    typedef typename GEOMETRY::DataVecCoord DataVecCoord;
//    typedef typename GEOMETRY::PROXIMITYDATA PROXIMITYDATA;
//    typedef TBaseNormalHandler<GEOMETRY> Inherit;

//    SOFA_CLASS(SOFA_TEMPLATE(EdgeNormalHandler,GEOMETRY), Inherit);

//    type::Vector3 computeNormal(const PROXIMITYDATA & data) const override {
//        const helper::ReadAccessor<DataVecCoord> & pos = this->l_geometry->getState()->read(core::VecCoordId::position());
//        return (pos[data.m_p1] - pos[data.m_p0]).normalized();
//    }

//    virtual void updateNormals() override {}
//};

//}

//}
