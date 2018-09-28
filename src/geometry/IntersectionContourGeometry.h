#pragma once

#include <BaseGeometry.h>
#include <geometry/PointGeometry.h>

namespace sofa {

namespace collisionAlgorithm {

class IntersectionContourGeometry : public PointGeometry {
public:
    Data<defaulttype::Vector3> d_planePos;
    Data<defaulttype::Vector3> d_planeNormal;

    IntersectionContourGeometry();

    void prepareDetection();

    void draw(const core::visual::VisualParams * vparams);

    std::vector<defaulttype::Vector3> m_pointNormal;
};

}

}
