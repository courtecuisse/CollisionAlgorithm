#pragma once

#include <geometry/PointGeometry.h>

namespace collisionAlgorithm {

class TriangleGeometry : public BaseGeometry {
    friend class TriangleElement;
    friend class TriangleProximity;

public:

    Port<Topology,REQUIRED> p_topology;
    Data<Vector4> d_color;

    TriangleGeometry()
    : p_topology("topology",LEFT,this)
    , d_color("color", Vector4(1,0,1,1), this){}

    void init();

    void prepareDetection();

    void draw(const VisualParams *vparams);

    void drawTriangle(const VisualParams * vparams,const Vector3 & A,const Vector3 & B, const Vector3 & C);

    typedef struct {
        Vector3 v0,v1;
        double d00;
        double d01;
        double d11;
        double invDenom;

        Vector3 tn,ax1,ax2;
    } TriangleInfo;

    inline ReadAccessor<Vector3> read(VecID v) {
        return p_topology->p_state->read(v);
    }

protected:
    std::vector<TriangleInfo> m_triangle_info;
    std::vector<Vector3> m_pointNormal;
};


}

