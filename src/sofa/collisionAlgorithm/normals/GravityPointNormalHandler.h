#pragma once

#include <sofa/collisionAlgorithm/geometry/PointGeometry.h>

namespace sofa {

namespace collisionAlgorithm {

template<class DataTypes>
class GravityPointNormalHandler : public PointGeometry<DataTypes>::PointNormalHandler {
public:
    SOFA_CLASS(SOFA_TEMPLATE(GravityPointNormalHandler,DataTypes), typename PointGeometry<DataTypes>::PointNormalHandler);

    GravityPointNormalHandler() {
        this->f_listening.setValue(true);
    }

    void init() override {
        PointGeometry<DataTypes>::PointNormalHandler::init();
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
        return (this->l_geometry->getPosition(data,core::VecCoordId::position()) - m_gcenter).normalized();
    }

private :
    defaulttype::Vector3 m_gcenter;
};

}

}
