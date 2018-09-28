#pragma once

#include <BaseGeometry.h>

namespace sofa {

namespace collisionAlgorithm {

class PointGeometry : public BaseGeometry {
    friend class PointElement;

public:
    Data<defaulttype::Vector4> d_color;

    PointGeometry()
    : d_color("color", defaulttype::Vector4(1,0,1,1), this){
        addActivateCondition(&PointGeometry::canCreate);
    }

    void prepareDetection();

    void init();

    sofa::core::behavior::BaseMechanicalState * getState() {
        return m_state;
    }

    bool canCreate() {
        getContext()->get(m_topology);
        getContext()->get(m_state);

        return m_state && m_topology;
    }

protected:
    core::topology::Topology * m_topology;
    core::behavior::BaseMechanicalState * m_state;
};

}

}
