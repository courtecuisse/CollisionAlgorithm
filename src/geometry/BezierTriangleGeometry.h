#pragma once

#include <geometry/TriangleGeometry.h>

namespace sofa {

namespace collisionAlgorithm {

class BezierTriangleGeometry : public TriangleGeometry {
    friend class BezierTriangleElement;
    friend class BezierTriangleProximity;

public:
    typedef TriangleGeometry Inherit;
    typedef typename Inherit::TriangleInfo TriangleInfo;

    Data <unsigned> d_nonlin_max_it;
    Data <double> d_nonlin_tolerance;
    Data <double> d_nonlin_threshold;
    Data <unsigned> d_draw_tesselation;

    BezierTriangleGeometry();

    void prepareDetection();

    void createElements();

private :
    typedef struct {
        defaulttype::Vector3 p210,p120,p021,p012,p102,p201,p111;
        defaulttype::Vector3 n110,n011,n101;
    } BezierTriangleInfo;

    std::vector<BezierTriangleInfo> m_beziertriangle_info;

};

}

}
