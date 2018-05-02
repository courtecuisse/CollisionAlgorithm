#pragma once

#include <geometry/TriangleGeometry.h>

namespace collisionAlgorithm {

class BezierTriangleGeometry : public TriangleGeometry {
public:
    typedef TriangleGeometry Inherit;
    typedef typename Inherit::TriangleInfo TriangleInfo;

    Data <unsigned> d_nonlin_max_it;
    Data <double> d_nonlin_tolerance;
    Data <double> d_nonlin_threshold;
    Data <unsigned> d_draw_tesselation;

    BezierTriangleGeometry();

    void prepareDetection();

    ConstraintProximityPtr getNonLinearTriangleProximity(unsigned eid, unsigned p1,double f1,unsigned p2, double f2, unsigned p3, double f3) const;

    ConstraintProximityPtr getElementProximity(unsigned eid) const;

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
