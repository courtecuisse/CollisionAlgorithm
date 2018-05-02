#pragma once

#include <geometry/BezierTriangleGeometry.h>
#include <element/BezierTriangleElement.h>

namespace collisionAlgorithm {

BezierTriangleGeometry::BezierTriangleGeometry()
: Inherit()
, d_nonlin_max_it("nonlin_max_it",(unsigned) 20, this)
, d_nonlin_tolerance("nonlin_tol",(double) 0.001,this)
, d_nonlin_threshold("nonlin_th",(double) 0.00001,this)
, d_draw_tesselation("tesselation",(unsigned) 0.0,this)
{}

void BezierTriangleGeometry::prepareDetection() {
    Inherit::prepareDetection();

    const ReadAccessor<Vector3> & x = read(VecCoordId::position());

    m_beziertriangle_info.resize(p_topology->getNbTriangles());
    for (unsigned t=0;t<p_topology()->getNbTriangles();t++) {
        BezierTriangleInfo & tbinfo = this->m_beziertriangle_info[t];
        const Topology::Triangle trpids = p_topology->getTriangle(t);

        const Vector3 & p300 = x[trpids[2]];
        const Vector3 & p030 = x[trpids[1]];
        const Vector3 & p003 = x[trpids[0]];

        const Vector3 & n200 = this->m_pointNormal[trpids[2]];
        const Vector3 & n020 = this->m_pointNormal[trpids[1]];
        const Vector3 & n002 = this->m_pointNormal[trpids[0]];

        double w12 = dot(p030 - p300,n200);
        double w21 = dot(p300 - p030,n020);
        double w23 = dot(p003 - p030,n020);
        double w32 = dot(p030 - p003,n002);
        double w31 = dot(p300 - p003,n002);
        double w13 = dot(p003 - p300,n200);

        tbinfo.p210 = (p300*2.0 + p030 - n200 * w12) / 3.0;
        tbinfo.p120 = (p030*2.0 + p300 - n020 * w21) / 3.0;

        tbinfo.p021 = (p030*2.0 + p003 - n020 * w23) / 3.0;
        tbinfo.p012 = (p003*2.0 + p030 - n002 * w32) / 3.0;

        tbinfo.p102 = (p003*2.0 + p300 - n002 * w31) / 3.0;
        tbinfo.p201 = (p300*2.0 + p003 - n200 * w13) / 3.0;

        Vector3 E = (tbinfo.p210+tbinfo.p120+tbinfo.p102+tbinfo.p201+tbinfo.p021+tbinfo.p012) / 6.0;
        Vector3 V = (p300+p030+p003) / 3.0;
        tbinfo.p111 =  E + (E-V) / 2.0;

        //Compute Bezier Normals
        double v12 = 2 * dot(p030-p300,n200+n020) / dot(p030-p300,p030-p300);
        double v23 = 2 * dot(p003-p030,n020+n002) / dot(p003-p030,p003-p030);
        double v31 = 2 * dot(p300-p003,n002+n200) / dot(p300-p003,p300-p003);

        Vector3 h110 = n200 + n020 - (p030-p300) * v12;
        Vector3 h011 = n020 + n002 - (p003-p030) * v23;
        Vector3 h101 = n002 + n200 - (p300-p003) * v31;

        tbinfo.n110 = h110 / h110.norm();
        tbinfo.n011 = h011 / h011.norm();
        tbinfo.n101 = h101 / h101.norm();
    }
}

void BezierTriangleGeometry::tesselate(const VisualParams * vparams, unsigned level,int tid, const Vector3 & bary_A,const Vector3 & bary_B, const Vector3 & bary_C) {
    if (level >= d_draw_tesselation.getValue()) {
        const Topology::Triangle & tri = p_topology->getTriangle(tid);

        Vector3 pA = getNonLinearTriangleProximity(tid,tri[0],bary_A[0],tri[1],bary_A[1],tri[2],bary_A[2])->getPosition();
        Vector3 pB = getNonLinearTriangleProximity(tid,tri[0],bary_B[0],tri[1],bary_B[1],tri[2],bary_B[2])->getPosition();
        Vector3 pC = getNonLinearTriangleProximity(tid,tri[0],bary_C[0],tri[1],bary_C[1],tri[2],bary_C[2])->getPosition();

        this->drawTriangle(vparams,pA,pB,pC);

        return;
    }

    Vector3 bary_D = (bary_A + bary_B)/2.0;
    Vector3 bary_E = (bary_A + bary_C)/2.0;
    Vector3 bary_F = (bary_B + bary_C)/2.0;

    Vector3 bary_G = (bary_A + bary_B + bary_C)/3.0;

    tesselate(vparams,level+1,tid,bary_A,bary_D,bary_G);
    tesselate(vparams,level+1,tid,bary_D,bary_B,bary_G);

    tesselate(vparams,level+1,tid,bary_G,bary_B,bary_F);
    tesselate(vparams,level+1,tid,bary_G,bary_F,bary_C);

    tesselate(vparams,level+1,tid,bary_G,bary_C,bary_E);
    tesselate(vparams,level+1,tid,bary_A,bary_G,bary_E);
}

void BezierTriangleGeometry::draw(const core::visual::VisualParams * vparams) {
        if (! vparams->displayFlags().getShowCollisionModels()) return;

        if (m_triangle_info.empty()) {
            Inherit::draw((vparams));
            return;
        }

        glDisable(GL_LIGHTING);

        if (vparams->displayFlags().getShowWireFrame()) glBegin(GL_LINES);
        else {
            glEnable(GL_CULL_FACE);
            glBegin(GL_TRIANGLES);
        }

        for(int t=0;t<this->getTopology()->getNbTriangles();t++) {
            tesselate(vparams,0,t,Vector3(1,0,0),Vector3(0,1,0),Vector3(0,0,1));
        }

        glEnd();
    }

}
