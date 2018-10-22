#pragma once

#include <sofa/collisionAlgorithm/geometry/BezierTriangleGeometry.h>
#include <sofa/collisionAlgorithm/element/TriangleElement.h>
#include <sofa/collisionAlgorithm/proximity/TriangleProximity.h>

namespace sofa {

namespace collisionAlgorithm {

class BezierTriangleElement : public TriangleElement {
    friend class BezierTriangleProximity;
public:

    BezierTriangleElement(BezierTriangleGeometry * geo, unsigned eid) : TriangleElement(geo,eid) {}

    ConstraintProximity::SPtr project(defaulttype::Vector3 P) const {
        return geometry()->newtonProject(this,P);
    }

    inline BezierTriangleGeometry * geometry() const {
        return (BezierTriangleGeometry*) m_geometry;
    }

    void tesselate(const core::visual::VisualParams * vparams, unsigned level,int tid, const defaulttype::Vector3 & bary_A,const defaulttype::Vector3 & bary_B, const defaulttype::Vector3 & bary_C) const {
        if (level >= geometry()->d_draw_tesselation.getValue()) {

            defaulttype::Vector3 pA = BezierTriangleGeometry::createProximity(this,bary_A[0],bary_A[1],bary_A[2])->getPosition();
            defaulttype::Vector3 pB = BezierTriangleGeometry::createProximity(this,bary_B[0],bary_B[1],bary_B[2])->getPosition();
            defaulttype::Vector3 pC = BezierTriangleGeometry::createProximity(this,bary_C[0],bary_C[1],bary_C[2])->getPosition();

            drawTriangle(vparams,pA,pB,pC);

            return;
        }

        defaulttype::Vector3 bary_D = (bary_A + bary_B)/2.0;
        defaulttype::Vector3 bary_E = (bary_A + bary_C)/2.0;
        defaulttype::Vector3 bary_F = (bary_B + bary_C)/2.0;

        defaulttype::Vector3 bary_G = (bary_A + bary_B + bary_C)/3.0;

        tesselate(vparams,level+1,tid,bary_A,bary_D,bary_G);
        tesselate(vparams,level+1,tid,bary_D,bary_B,bary_G);

        tesselate(vparams,level+1,tid,bary_G,bary_B,bary_F);
        tesselate(vparams,level+1,tid,bary_G,bary_F,bary_C);

        tesselate(vparams,level+1,tid,bary_G,bary_C,bary_E);
        tesselate(vparams,level+1,tid,bary_A,bary_G,bary_E);
    }

    virtual void draw(const core::visual::VisualParams *vparams) const {
        tesselate(vparams,0,m_eid,defaulttype::Vector3(1,0,0),defaulttype::Vector3(0,1,0),defaulttype::Vector3(0,0,1));
    }
};

}

}
