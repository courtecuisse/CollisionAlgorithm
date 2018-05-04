#pragma once

#include <BaseGeometry.h>
#include <geometry/PointGeometry.h>

namespace collisionAlgorithm {

class IntersectionContourGeometry : public PointGeometry {
public:
    Data<Vector3> d_planePos;
    Data<Vector3> d_planeNormal;

    IntersectionContourGeometry();

    void prepareDetection();

    void draw(const VisualParams * vparams);

    std::vector<Vector3> m_pointNormal;
};

}
