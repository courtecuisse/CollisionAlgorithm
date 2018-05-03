#pragma once

#include <geometry/TriangleGeometry.h>

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

    void draw(const VisualParams * vparams);

private :
    typedef struct {
        Vector3 p210,p120,p021,p012,p102,p201,p111;
        Vector3 n110,n011,n101;
    } BezierTriangleInfo;

    std::vector<BezierTriangleInfo> m_beziertriangle_info;

    void tesselate(const VisualParams * vparams, unsigned level,int tid, const Vector3 & bary_A,const Vector3 & bary_B, const Vector3 & bary_C);

};

}
