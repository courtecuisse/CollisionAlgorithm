#pragma once

#include <sofa/collisionAlgorithm/geometry/PointGeometry.h>

namespace sofa {

namespace collisionAlgorithm {

template<class DataTypes>
class GravityPointNormalHandler : public TBaseNormalHandler<DataTypes,PointProximity> {
public:
    typedef typename DataTypes::VecCoord VecCoord;
    typedef core::objectmodel::Data< VecCoord >        DataVecCoord;

    SOFA_CLASS(SOFA_TEMPLATE(GravityPointNormalHandler,DataTypes), SOFA_TEMPLATE2(TBaseNormalHandler,DataTypes,PointProximity));

    GravityPointNormalHandler() {
        this->f_listening.setValue(true);
    }

    void init() override {
        TBaseNormalHandler<DataTypes,PointProximity>::init();
        computeGravityCenter();
    }

    void handleEvent(sofa::core::objectmodel::Event* event) override {
        if (dynamic_cast<sofa::simulation::AnimateBeginEvent*>(event)) computeGravityCenter();
    }

    void computeGravityCenter() {
        m_gcenter = defaulttype::Vector3();
        unsigned size = 0;
        for (auto it = this->l_geometry->begin();it!=this->l_geometry->end();it++) {
            m_gcenter += it->center()->getPosition();
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
