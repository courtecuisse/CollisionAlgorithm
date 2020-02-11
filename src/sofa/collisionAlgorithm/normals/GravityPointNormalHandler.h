#pragma once

#include <sofa/collisionAlgorithm/BaseNormalHandler.h>

namespace sofa {

namespace collisionAlgorithm {

template<class GEOMETRY>
class GravityPointNormalHandler : public TBaseNormalHandler<GEOMETRY> {
public:
    typedef typename GEOMETRY::VecCoord VecCoord;
    typedef typename GEOMETRY::DataVecCoord DataVecCoord;
    typedef typename GEOMETRY::PROXIMITYDATA PROXIMITYDATA;
    typedef TBaseNormalHandler<GEOMETRY> Inherit;

    SOFA_CLASS(SOFA_TEMPLATE(GravityPointNormalHandler,GEOMETRY), Inherit);

    GravityPointNormalHandler() {
        this->f_listening.setValue(true);
    }

    virtual void updateNormals() override {
        m_gcenter = defaulttype::Vector3();
        unsigned size = 0;
        for (auto it = this->l_geometry->begin();it!=this->l_geometry->end();it++) {
            m_gcenter += it->createProximity()->getPosition();
            size++;
        }

        if (size != 0) m_gcenter*=1.0/size;
    }


    defaulttype::Vector3 computeNormal(const PointProximity & data) const override {
        const helper::ReadAccessor<DataVecCoord> & pos = this->l_geometry->getState()->read(core::VecCoordId::position());
        return (pos[data.m_eid] - m_gcenter).normalized();
    }

private :
    defaulttype::Vector3 m_gcenter;
};

}

}
