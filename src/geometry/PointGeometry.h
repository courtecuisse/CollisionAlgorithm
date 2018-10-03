#pragma once

#include <BaseGeometry.h>
#include <sofa/core/topology/BaseMeshTopology.h>

namespace sofa {

namespace collisionAlgorithm {

class PointGeometry : public BaseGeometry {
    friend class PointElement;

public:
    Data<defaulttype::Vector4> d_color;
    DataLink<core::topology::BaseMeshTopology> d_topology;

    PointGeometry()
    : d_color(initData(&d_color, defaulttype::Vector4(1,0,1,1), "color", "Color of the collision model"))
    , d_topology("topology", this) {
        d_topology.addCallback(&PointGeometry::updateTopology);
    }

    void updateTopology() {
        if (d_topology!=NULL && d_state!=NULL) initialize();
    }

    void prepareDetection();

    void initialize();
};

}

}
