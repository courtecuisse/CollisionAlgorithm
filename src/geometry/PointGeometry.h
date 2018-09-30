#pragma once

#include <BaseGeometry.h>
#include <sofa/core/topology/BaseMeshTopology.h>

namespace sofa {

namespace collisionAlgorithm {

class PointGeometry : public BaseGeometry {
    friend class PointElement;

public:
    Data<defaulttype::Vector4> d_color;

    PointGeometry()
    : d_color(initData(&d_color, defaulttype::Vector4(1,0,1,1), "color", "Color of the collision model")) {
        addActivateCondition(&PointGeometry::canCreate);
    }

    void prepareDetection();

    void init();

    sofa::core::behavior::MechanicalState<defaulttype::Vec3dTypes> * getState() {
        return m_state;
    }

    bool canCreate() {
        getContext()->get(m_topology);
        getContext()->get(m_state);

        return m_state && m_topology;
    }

protected:
    core::topology::BaseMeshTopology * m_topology;
    sofa::core::behavior::MechanicalState<defaulttype::Vec3dTypes> * m_state;
};

}

}
