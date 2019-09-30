#pragma once

#include <sofa/collisionAlgorithm/geometry/PhongTriangleGeometry.h>

namespace sofa
{

namespace collisionAlgorithm
{

template<class DataTypes>
class BezierTriangleGeometry : public PhongTriangleGeometry<DataTypes> {
public:
    typedef DataTypes TDataTypes;
    typedef PhongTriangleGeometry<DataTypes> Inherit;
    typedef BezierTriangleGeometry<DataTypes> GEOMETRY;
    typedef typename DataTypes::VecCoord VecCoord;
    typedef core::objectmodel::Data< VecCoord >        DataVecCoord;
    typedef size_t TriangleID;
    typedef sofa::core::topology::BaseMeshTopology::Triangle Triangle;

    SOFA_CLASS(GEOMETRY,Inherit);

    typedef struct
    {
        defaulttype::Vector3 p210,p120,p021,p012,p102,p201,p111;
        defaulttype::Vector3 n110,n011,n101;
    } BezierTriangleInfo;

    Data <unsigned> d_nonlin_max_it;
    Data <double> d_nonlin_tolerance;
    Data <double> d_nonlin_threshold;
    Data <unsigned> d_draw_tesselation;

    BezierTriangleGeometry()
    : d_nonlin_max_it(initData(&d_nonlin_max_it,(unsigned) 20,"nonlin_max_it", "number of iterations"))
    , d_nonlin_tolerance(initData(&d_nonlin_tolerance,(double) 0.001,"nonlin_tol", "Tolerance"))
    , d_nonlin_threshold(initData(&d_nonlin_threshold,(double) 0.00001,"nonlin_th", "Threshold"))
    , d_draw_tesselation(initData(&d_draw_tesselation,(unsigned) 0.0, "tesselation", "Number of tesselation")) {}

    inline BaseElementIterator::UPtr begin(unsigned eid = 0) const override {
        return DefaultElementIterator<TriangleProximity>::create(this,this->l_topology->getTriangles(),eid);
    }

    virtual void prepareDetection() {
        Inherit::prepareDetection();

        const helper::vector<Triangle>& triangles = this->l_topology->getTriangles();
        const helper::ReadAccessor<DataVecCoord> & x = this->getState()->read(core::VecCoordId::position());

        unsigned nbTriangles = triangles.size();
        m_beziertriangle_info.resize(nbTriangles);
        for (size_t t=0;t< nbTriangles;t++)
        {
            BezierTriangleInfo & tbinfo = this->m_beziertriangle_info[t];
            const Triangle& trpids = triangles[t];

            const defaulttype::Vector3 & p300 = x[trpids[2]];
            const defaulttype::Vector3 & p030 = x[trpids[1]];
            const defaulttype::Vector3 & p003 = x[trpids[0]];

            const defaulttype::Vector3 & n200 = this->m_point_normals[trpids[2]];
            const defaulttype::Vector3 & n020 = this->m_point_normals[trpids[1]];
            const defaulttype::Vector3 & n002 = this->m_point_normals[trpids[0]];

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

            defaulttype::Vector3 E = (tbinfo.p210+tbinfo.p120+tbinfo.p102+tbinfo.p201+tbinfo.p021+tbinfo.p012) / 6.0;
            defaulttype::Vector3 V = (p300+p030+p003) / 3.0;
            tbinfo.p111 =  E + (E-V) / 2.0;

            //Compute Bezier Normals
            double v12 = 2 * dot(p030-p300,n200+n020) / dot(p030-p300,p030-p300);
            double v23 = 2 * dot(p003-p030,n020+n002) / dot(p003-p030,p003-p030);
            double v31 = 2 * dot(p300-p003,n002+n200) / dot(p300-p003,p300-p003);

            defaulttype::Vector3 h110 = n200 + n020 - (p030-p300) * v12;
            defaulttype::Vector3 h011 = n020 + n002 - (p003-p030) * v23;
            defaulttype::Vector3 h101 = n002 + n200 - (p300-p003) * v31;

            tbinfo.n110 = h110 / h110.norm();
            tbinfo.n011 = h011 / h011.norm();
            tbinfo.n101 = h101 / h101.norm();
        }
    }

    void tesselate(unsigned level,int tid, const defaulttype::Vector3 & bary_A,const defaulttype::Vector3 & bary_B, const defaulttype::Vector3 & bary_C) {
        if (level >= d_draw_tesselation.getValue()) {

            const Triangle& triangle = this->l_topology->getTriangle(tid);

            TriangleProximity proxA(tid, triangle[0],triangle[1],triangle[2], bary_A[0],bary_A[1],bary_A[2]);
            TriangleProximity proxB(tid, triangle[0],triangle[1],triangle[2], bary_B[0],bary_B[1],bary_B[2]);
            TriangleProximity proxC(tid, triangle[0],triangle[1],triangle[2], bary_C[0],bary_C[1],bary_C[2]);

            // draw Triangle
            double delta = 0.2;

            const defaulttype::Vector4 & color = this->d_color.getValue();

            glColor4f(fabs(color[0]-delta),color[1],color[2],color[3]);
            glVertex3dv(getPosition(proxA).data());
            glColor4f(color[0],fabs(color[1]-delta),color[2],color[3]);
            glVertex3dv(getPosition(proxB).data());
            glColor4f(color[0],color[1],fabs(color[2]-delta),color[3]);
            glVertex3dv(getPosition(proxC).data());

            return;
        }

        defaulttype::Vector3 bary_D = (bary_A + bary_B)/2.0;
        defaulttype::Vector3 bary_E = (bary_A + bary_C)/2.0;
        defaulttype::Vector3 bary_F = (bary_B + bary_C)/2.0;

        defaulttype::Vector3 bary_G = (bary_A + bary_B + bary_C)/3.0;

        tesselate(level+1,tid,bary_A,bary_D,bary_G);
        tesselate(level+1,tid,bary_D,bary_B,bary_G);

        tesselate(level+1,tid,bary_G,bary_B,bary_F);
        tesselate(level+1,tid,bary_G,bary_F,bary_C);

        tesselate(level+1,tid,bary_G,bary_C,bary_E);
        tesselate(level+1,tid,bary_A,bary_G,bary_E);
    }

    void draw(const core::visual::VisualParams * vparams) {
        this->drawNormals(vparams);

//        if (! vparams->displayFlags().getShowCollisionModels()) return;
        if (! vparams->displayFlags().getShowCollisionModels()) return ;

        const defaulttype::Vector4 & color = this->d_color.getValue();
        if (color[3] == 0.0) return;

        glDisable(GL_LIGHTING);

        glBegin(GL_TRIANGLES);
        for (auto it = this->begin();it != this->end();it++) {
            tesselate(0, it->id() , defaulttype::Vector3(1,0,0),defaulttype::Vector3(0,1,0),defaulttype::Vector3(0,0,1));
        }
        glEnd();
    }

    inline TriangleProximity project(unsigned eid, const Triangle & triangle, const defaulttype::Vector3 & P) const {
        unsigned max_it = d_nonlin_max_it.getValue();
        double tolerance = d_nonlin_tolerance.getValue();
        double threshold = d_nonlin_threshold.getValue();

        unsigned int it=0;
        double delta = 0.00001;

        //initialize the algorithm xith the projection on a linear triangle
        TriangleProximity pinfo = Inherit::project(eid,triangle,P);

        while(it< max_it)
        {
            defaulttype::Vector3 Q = getPosition(pinfo);

            defaulttype::Vector3 nQP = P - Q;
            if (nQP.norm() < tolerance) break;
            nQP.normalize();

            defaulttype::Vector3 N1 = getNormal(pinfo);
            N1.normalize();

            if (pinfo.m_f0 < 0 || pinfo.m_f1 < 0 || pinfo.m_f2 < 0) break;

            defaulttype::Vector3 N2 = cross(N1,((fabs(dot(N1,defaulttype::Vector3(1,0,0)))>0.99) ? defaulttype::Vector3(0,1,0) : defaulttype::Vector3(1,0,0)));
            N2.normalize();

            defaulttype::Vector3 N3 = cross(N1,N2);
            N3.normalize();

            defaulttype::Vector2 e_0(dot(nQP,N2),dot(nQP,N3));

            if(e_0.norm() < tolerance) break;

            double fact_u = (pinfo.m_f2 - delta < 0.0 || pinfo.m_f1 + delta > 1.0) ? -1.0 : 1.0;
            double fact_v = (pinfo.m_f2 - delta < 0.0 || pinfo.m_f0 + delta > 1.0) ? -1.0 : 1.0;

            //variation point along v
            double P_v_fact0 = pinfo.m_f0 + delta * fact_v;
            double P_v_fact1 = pinfo.m_f1;
            double P_v_fact2 = pinfo.m_f2 - delta * fact_v;
            if (P_v_fact0 < 0 || P_v_fact1 < 0 || P_v_fact2 < 0) break;

            TriangleProximity P_v(eid, pinfo.m_p0,pinfo.m_p1,pinfo.m_p2, P_v_fact0, P_v_fact1, P_v_fact2);
            defaulttype::Vector3 p_v = (P - getPosition(P_v)).normalized();
            defaulttype::Vector2 e_v(dot(p_v,N2)*fact_v,dot(p_v,N3)*fact_v);

            //variation point along u
            double P_u_fact0 = pinfo.m_f0;
            double P_u_fact1 = pinfo.m_f1 + delta * fact_u;
            double P_u_fact2 = pinfo.m_f2 - delta * fact_u;
            if (P_u_fact0 < 0 || P_u_fact1 < 0 || P_u_fact2 < 0) break;

            TriangleProximity P_u(eid, pinfo.m_p0,pinfo.m_p1,pinfo.m_p2, P_u_fact0,P_u_fact1,P_u_fact2);
            defaulttype::Vector3 p_u = (P - getPosition(P_u)).normalized();
            defaulttype::Vector2 e_u(dot(p_u,N2)*fact_u,dot(p_u,N3)*fact_u);

            defaulttype::Mat2x2d J, invJ;
            J[0][0] = (e_v[0] - e_0[0])/delta;
            J[1][0] = (e_v[1] - e_0[1])/delta;
            J[0][1] = (e_u[0] - e_0[0])/delta;
            J[1][1] = (e_u[1] - e_0[1])/delta;

            invertMatrix(invJ, J);

            // dUV is the optimal direction
            defaulttype::Vector2 dUV = -invJ * e_0;
            if(dUV.norm() < threshold) break;

            //bary coords of the solution of the 2D problem
            double sol_v = pinfo.m_f0 + dUV[0];
            double sol_u = pinfo.m_f1 + dUV[1];
            double sol_w = 1.0 - sol_u - sol_v;

            // we now search what is the optimal displacmeent along this path
            defaulttype::Vector3 dir2d(sol_v - pinfo.m_f0,
                                       sol_u - pinfo.m_f1,
                                       sol_w - pinfo.m_f2);

            if(dir2d.norm() < threshold) break;

            //we apply a small perturbation arond the 2d direction
            dir2d.normalize();

            double fact_a = (pinfo.m_f0 + dir2d[0] * delta < 0 || pinfo.m_f1 + dir2d[1] * delta < 0 || pinfo.m_f2 + dir2d[2] * delta < 0) ? -1.0 : 1.0;
            double P_a_fact0 = pinfo.m_f0 + dir2d[0] * delta * fact_a;
            double P_a_fact1 = pinfo.m_f1 + dir2d[1] * delta * fact_a;
            double P_a_fact2 = pinfo.m_f2 + dir2d[2] * delta * fact_a;

            if (P_a_fact0 < 0 ||P_a_fact1 < 0 || P_a_fact2 < 0) break;

            TriangleProximity P_a(eid, pinfo.m_p0,pinfo.m_p1,pinfo.m_p2, P_a_fact0,P_a_fact1,P_a_fact2);
            defaulttype::Vector3 QA = getPosition(P_a);

            double fact;
            if (fabs(dot(nQP,N1))>0.8) {
                double fx = acos(fabs(dot(nQP,N1)));
                double fxdx = acos(fabs(dot((P - QA).normalized(),getNormal(P_a))));
                double j = (fxdx - fx) / delta;
                fact = -fx / j;
            } else {
                defaulttype::Vector3 nQA = (Q-QA).normalized();
                double fx = dot(P-Q, nQA);
                double fxdx = dot(P-QA, nQA);
                double j = (fxdx - fx) / delta;
                fact = -fx / j;
            }

            if(fabs(fact) < threshold) break;

            dir2d *= fact * fact_a;

            double new_v = pinfo.m_f0 + dir2d[0];
            double new_u = pinfo.m_f1 + dir2d[1];
            double new_w = pinfo.m_f2 + dir2d[2];

            if (new_v<0 && fabs(dir2d[0])>0) dir2d *= -pinfo.m_f0 / dir2d[0];
            if (new_u<0 && fabs(dir2d[1])>0) dir2d *= -pinfo.m_f1 / dir2d[1];
            if (new_w<0 && fabs(dir2d[2])>0) dir2d *= -pinfo.m_f2 / dir2d[2];

            pinfo.m_f0 += dir2d[0];
            pinfo.m_f1 += dir2d[1];
            pinfo.m_f2 += dir2d[2];

            it++;
        }

        return pinfo;
    }

    inline defaulttype::Vector3 getNormal(const TriangleProximity & data) const {
        const defaulttype::Vector3 &n200 = this->m_triangle_normals[data.m_p2];
        const defaulttype::Vector3 &n020 = this->m_triangle_normals[data.m_p1];
        const defaulttype::Vector3 &n002 = this->m_triangle_normals[data.m_p0];
        const BezierTriangleInfo & tbinfo = this->m_beziertriangle_info[data.m_eid];

        double fact_w = data.m_f2;
        double fact_u = data.m_f1;
        double fact_v = data.m_f0;

        defaulttype::Vector3 normal = n200 * fact_w*fact_w +
                                      n020 * fact_u*fact_u +
                                      n002 * fact_v*fact_v +
                                      tbinfo.n110 * fact_w*fact_u +
                                      tbinfo.n011 * fact_u*fact_v +
                                      tbinfo.n101 * fact_w*fact_v;

        return normal.normalized();
    }

    ////Bezier triangle are computed according to :
    ////http://www.gamasutra.com/view/feature/131389/b%C3%A9zier_triangles_and_npatches.php?print=1
    inline defaulttype::Vector3 getPosition(const TriangleProximity & data, core::VecCoordId v = core::VecCoordId::position()) const {
        const BezierTriangleInfo & tbinfo = this->m_beziertriangle_info[data.m_eid];

        if(v == core::VecCoordId::position())
        {
            const helper::ReadAccessor<DataVecCoord> & x = this->getState()->read(v);

            const defaulttype::Vector3 & p300 = x[data.m_p2];
            const defaulttype::Vector3 & p030 = x[data.m_p1];
            const defaulttype::Vector3 & p003 = x[data.m_p0];

            double fact_w = data.m_f2;
            double fact_u = data.m_f1;
            double fact_v = data.m_f0;

            return p300 *   fact_w*fact_w*fact_w +
                   p030 *   fact_u*fact_u*fact_u +
                   p003 *   fact_v*fact_v*fact_v +
                   tbinfo.p210 * 3*fact_w*fact_w*fact_u +
                   tbinfo.p120 * 3*fact_w*fact_u*fact_u +
                   tbinfo.p201 * 3*fact_w*fact_w*fact_v +
                   tbinfo.p021 * 3*fact_u*fact_u*fact_v +
                   tbinfo.p102 * 3*fact_w*fact_v*fact_v +
                   tbinfo.p012 * 3*fact_u*fact_v*fact_v +
                   tbinfo.p111 * 6*fact_w*fact_u*fact_v;
        } else {
            double fact_w = data.m_f2;
            double fact_u = data.m_f1;
            double fact_v = data.m_f0;

            const helper::ReadAccessor<DataVecCoord> & x = this->getState()->read(v);

            const defaulttype::Vector3 & p300_Free = x[data.m_p2];
            const defaulttype::Vector3 & p030_Free = x[data.m_p1];
            const defaulttype::Vector3 & p003_Free = x[data.m_p0];

            const defaulttype::Vector3 & n200_Free = this->m_triangle_normals[data.m_p2];
            const defaulttype::Vector3 & n020_Free = this->m_triangle_normals[data.m_p1];
            const defaulttype::Vector3 & n002_Free = this->m_triangle_normals[data.m_p0];

            double w12_free = dot(p030_Free - p300_Free,n200_Free);
            double w21_free = dot(p300_Free - p030_Free,n020_Free);
            double w23_free = dot(p003_Free - p030_Free,n020_Free);
            double w32_free = dot(p030_Free - p003_Free,n002_Free);
            double w31_free = dot(p300_Free - p003_Free,n002_Free);
            double w13_free = dot(p003_Free - p300_Free,n200_Free);

            const defaulttype::Vector3 & p210_Free = (p300_Free*2.0 + p030_Free - n200_Free * w12_free) / 3.0;
            const defaulttype::Vector3 & p120_Free = (p030_Free*2.0 + p300_Free - n020_Free * w21_free) / 3.0;

            const defaulttype::Vector3 & p021_Free = (p030_Free*2.0 + p003_Free - n020_Free * w23_free) / 3.0;
            const defaulttype::Vector3 & p012_Free = (p003_Free*2.0 + p030_Free - n002_Free * w32_free) / 3.0;

            const defaulttype::Vector3 & p102_Free = (p003_Free*2.0 + p300_Free - n002_Free * w31_free) / 3.0;
            const defaulttype::Vector3 & p201_Free = (p300_Free*2.0 + p003_Free - n200_Free * w13_free) / 3.0;

            const defaulttype::Vector3 & E_Free = (p210_Free+p120_Free+p102_Free+p201_Free+p021_Free+p012_Free) / 6.0;
            const defaulttype::Vector3 & V_Free = (p300_Free+p030_Free+p003_Free) / 3.0;
            const defaulttype::Vector3 & p111_Free =  E_Free + (E_Free-V_Free) / 2.0;

            return p300_Free *   fact_w*fact_w*fact_w +
                   p030_Free *   fact_u*fact_u*fact_u +
                   p003_Free *   fact_v*fact_v*fact_v +
                   p210_Free * 3*fact_w*fact_w*fact_u +
                   p120_Free * 3*fact_w*fact_u*fact_u +
                   p201_Free * 3*fact_w*fact_w*fact_v +
                   p021_Free * 3*fact_u*fact_u*fact_v +
                   p102_Free * 3*fact_w*fact_v*fact_v +
                   p012_Free * 3*fact_u*fact_v*fact_v +
                   p111_Free * 6*fact_w*fact_u*fact_v;
        }
    }

protected:
    std::vector<BezierTriangleInfo> m_beziertriangle_info;
};

}

}
