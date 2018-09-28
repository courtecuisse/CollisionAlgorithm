#pragma once

#include <geometry/PointGeometry.h>
#include <geometry/EdgeGeometry.h>

namespace sofa {

namespace collisionAlgorithm {

class TriangleGeometry : public EdgeGeometry {
    friend class TriangleElement;
    friend class TriangleProximity;

public:

    virtual void init();

    virtual void prepareDetection();

    typedef struct {
        defaulttype::Vector3 v0,v1;
        double d00;
        double d01;
        double d11;
        double invDenom;

        defaulttype::Vector3 tn,ax1,ax2;
    } TriangleInfo;

protected:
    std::vector<TriangleInfo> m_triangle_info;
    std::vector<defaulttype::Vector3> m_pointNormal;
};


}

}
