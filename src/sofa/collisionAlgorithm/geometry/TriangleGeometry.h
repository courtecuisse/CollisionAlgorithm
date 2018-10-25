#pragma once

#include <sofa/collisionAlgorithm/geometry/PointGeometry.h>
#include <sofa/collisionAlgorithm/geometry/EdgeGeometry.h>

namespace sofa {

namespace collisionAlgorithm {

class TriangleElement;

class TriangleGeometry : public EdgeGeometry {
    friend class TriangleElement;
    friend class TriangleProximity;

public:
    SOFA_CLASS(TriangleGeometry,EdgeGeometry);

    virtual void init();

    virtual void prepareDetection();

    static ConstraintProximity::SPtr createProximity(const TriangleElement * elmt,double f1,double f2,double f3);

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
