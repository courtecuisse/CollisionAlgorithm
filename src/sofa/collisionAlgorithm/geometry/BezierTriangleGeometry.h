#pragma once

#include <sofa/collisionAlgorithm/geometry/TriangleGeometry.h>

namespace sofa
{

namespace collisionAlgorithm
{

template<class DataTypes>
class BezierTriangleGeometry : public TBaseGeometry<DataTypes,TriangleProximity> {
public:
    typedef DataTypes TDataTypes;
    typedef TBaseGeometry<DataTypes,TriangleProximity> Inherit;
    typedef BaseProximity::Index Index;
    typedef BezierTriangleGeometry<DataTypes> GEOMETRY;
    typedef typename Inherit::PROXIMITYDATA PROXIMITYDATA;
    typedef typename DataTypes::VecCoord VecCoord;
    typedef core::objectmodel::Data< VecCoord >        DataVecCoord;
    typedef size_t TriangleID;
    typedef sofa::topology::Triangle Triangle;
    typedef sofa::type::vector<Triangle> VecTriangles;

    SOFA_CLASS(GEOMETRY,Inherit);

    typedef struct
    {
        type::Vector3 p300,p030,p003;
        type::Vector3 p210,p120,p021,p012,p102,p201,p111;
        type::Vector3 n110,n011,n101;
        type::Vector3 n200,n020,n002;
    } BezierTriangleInfo;

    core::objectmodel::SingleLink<GEOMETRY,core::topology::BaseMeshTopology,BaseLink::FLAG_STRONGLINK|BaseLink::FLAG_STOREPATH> l_topology;

    Data <Index> d_nonlin_max_it;
    Data <double> d_nonlin_tolerance;
    Data <double> d_nonlin_threshold;
    Data <Index> d_draw_tesselation;

    BezierTriangleGeometry()
    : l_topology(initLink("topology", "link to topology"))
    , d_nonlin_max_it(initData(&d_nonlin_max_it,(Index) 20,"nonlin_max_it", "number of iterations"))
    , d_nonlin_tolerance(initData(&d_nonlin_tolerance,(double) 0.001,"nonlin_tol", "Tolerance"))
    , d_nonlin_threshold(initData(&d_nonlin_threshold,(double) 0.00001,"nonlin_th", "Threshold"))
    , d_draw_tesselation(initData(&d_draw_tesselation,(Index) 0.0, "tesselation", "Number of tesselation")) {
            l_topology.setPath("@.");
    }

    inline BaseElementIterator::UPtr begin(Index eid = 0) const override {
        return DefaultElementIterator<PROXIMITYDATA>::create(this,this->l_topology->getTriangles(),eid);
    }

    virtual void prepareDetection() {
        m_bezier_info.clear();
    }

    void tesselate(Index level,int tid, const type::Vector3 & bary_A,const type::Vector3 & bary_B, const type::Vector3 & bary_C) {
        if (level >= d_draw_tesselation.getValue()) {

            const Triangle& triangle = this->l_topology->getTriangle(tid);

            PROXIMITYDATA proxA(tid, triangle[0],triangle[1],triangle[2], bary_A[0],bary_A[1],bary_A[2]);
            PROXIMITYDATA proxB(tid, triangle[0],triangle[1],triangle[2], bary_B[0],bary_B[1],bary_B[2]);
            PROXIMITYDATA proxC(tid, triangle[0],triangle[1],triangle[2], bary_C[0],bary_C[1],bary_C[2]);

            // draw Triangle
            double delta = 0.2;

            const sofa::type::RGBAColor & color = this->d_color.getValue();

            glColor4f(fabs(color[0]-delta),color[1],color[2],color[3]);
            glVertex3dv(getPosition(proxA).data());
            glColor4f(color[0],fabs(color[1]-delta),color[2],color[3]);
            glVertex3dv(getPosition(proxB).data());
            glColor4f(color[0],color[1],fabs(color[2]-delta),color[3]);
            glVertex3dv(getPosition(proxC).data());

            return;
        }

        type::Vector3 bary_D = (bary_A + bary_B)/2.0;
        type::Vector3 bary_E = (bary_A + bary_C)/2.0;
        type::Vector3 bary_F = (bary_B + bary_C)/2.0;

        type::Vector3 bary_G = (bary_A + bary_B + bary_C)/3.0;

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

        const sofa::type::RGBAColor & color = this->d_color.getValue();
        if (color[3] == 0.0) return;

        glDisable(GL_LIGHTING);

        glBegin(GL_TRIANGLES);
        for (auto it = this->begin();it != this->end();it++) {
            tesselate(0, it->id() , type::Vector3(1,0,0),type::Vector3(0,1,0),type::Vector3(0,0,1));
        }
        glEnd();
    }

    inline const sofa::topology::Triangle getTriangle(Index eid) const {
        return this->l_topology->getTriangle(eid);
    }

    inline PROXIMITYDATA project(const type::Vector3 & P, Index eid) const {
        auto triangle = getTriangle(eid);

        Index max_it = d_nonlin_max_it.getValue();
        double tolerance = d_nonlin_tolerance.getValue();
        double threshold = d_nonlin_threshold.getValue();

        Index it=0;
        double delta = 0.00001;

        //initialize the algorithm xith the projection on a linear triangle

        auto tinfo = getBezierInfo()[eid];
        double fact_u,fact_v,fact_w;
        toolBox::projectOnTriangle(P,
                                   toolBox::computeTriangleInfo(tinfo.p300,tinfo.p030,tinfo.p003),
                                   fact_u,fact_v,fact_w);

        auto pinfo = PROXIMITYDATA(eid,triangle[0],triangle[1],triangle[2],fact_u,fact_v,fact_w);

        while(it< max_it) {
            type::Vector3 Q = getPosition(pinfo);

            type::Vector3 nQP = P - Q;
            if (nQP.norm() < tolerance) break;
            nQP.normalize();

            type::Vector3 N1 = this->getNormal(pinfo);
            N1.normalize();

            if (pinfo.m_f0 < 0 || pinfo.m_f1 < 0 || pinfo.m_f2 < 0) break;

            type::Vector3 N2 = cross(N1,((fabs(dot(N1,type::Vector3(1,0,0)))>0.99) ? type::Vector3(0,1,0) : type::Vector3(1,0,0)));
            N2.normalize();

            type::Vector3 N3 = cross(N1,N2);
            N3.normalize();

            type::Vector2 e_0(dot(nQP,N2),dot(nQP,N3));

            if(e_0.norm() < tolerance) break;

            double fact_u = (pinfo.m_f2 - delta < 0.0 || pinfo.m_f1 + delta > 1.0) ? -1.0 : 1.0;
            double fact_v = (pinfo.m_f2 - delta < 0.0 || pinfo.m_f0 + delta > 1.0) ? -1.0 : 1.0;

            //variation point along v
            double P_v_fact0 = pinfo.m_f0 + delta * fact_v;
            double P_v_fact1 = pinfo.m_f1;
            double P_v_fact2 = pinfo.m_f2 - delta * fact_v;
            if (P_v_fact0 < 0 || P_v_fact1 < 0 || P_v_fact2 < 0) break;

            PROXIMITYDATA P_v(eid, pinfo.m_p0,pinfo.m_p1,pinfo.m_p2, P_v_fact0, P_v_fact1, P_v_fact2);
            type::Vector3 p_v = (P - getPosition(P_v)).normalized();
            type::Vector2 e_v(dot(p_v,N2)*fact_v,dot(p_v,N3)*fact_v);

            //variation point along u
            double P_u_fact0 = pinfo.m_f0;
            double P_u_fact1 = pinfo.m_f1 + delta * fact_u;
            double P_u_fact2 = pinfo.m_f2 - delta * fact_u;
            if (P_u_fact0 < 0 || P_u_fact1 < 0 || P_u_fact2 < 0) break;

            PROXIMITYDATA P_u(eid, pinfo.m_p0,pinfo.m_p1,pinfo.m_p2, P_u_fact0,P_u_fact1,P_u_fact2);
            type::Vector3 p_u = (P - getPosition(P_u)).normalized();
            type::Vector2 e_u(dot(p_u,N2)*fact_u,dot(p_u,N3)*fact_u);

            type::Mat2x2d J, invJ;
            J[0][0] = (e_v[0] - e_0[0])/delta;
            J[1][0] = (e_v[1] - e_0[1])/delta;
            J[0][1] = (e_u[0] - e_0[0])/delta;
            J[1][1] = (e_u[1] - e_0[1])/delta;

            invertMatrix(invJ, J);

            // dUV is the optimal direction
            type::Vector2 dUV = -invJ * e_0;
            if(dUV.norm() < threshold) break;

            //bary coords of the solution of the 2D problem
            double sol_v = pinfo.m_f0 + dUV[0];
            double sol_u = pinfo.m_f1 + dUV[1];
            double sol_w = 1.0 - sol_u - sol_v;

            // we now search what is the optimal displacmeent along this path
            type::Vector3 dir2d(sol_v - pinfo.m_f0,
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

            PROXIMITYDATA P_a(eid, pinfo.m_p0,pinfo.m_p1,pinfo.m_p2, P_a_fact0,P_a_fact1,P_a_fact2);
            type::Vector3 QA = getPosition(P_a);

            double fact;
            if (fabs(dot(nQP,N1))>0.8) {
                double fx = acos(fabs(dot(nQP,N1)));
                double fxdx = acos(fabs(dot((P - QA).normalized(),this->getNormal(P_a))));
                double j = (fxdx - fx) / delta;
                fact = -fx / j;
            } else {
                type::Vector3 nQA = (Q-QA).normalized();
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

    // Force use of the function to compute the normal (not the normal handler)
    type::Vector3 computeNormal(const PROXIMITYDATA & data) const override {
        auto tbinfo = getBezierInfo()[data.m_eid];

        const type::Vector3 &n200 = tbinfo.n200;
        const type::Vector3 &n020 = tbinfo.n020;
        const type::Vector3 &n002 = tbinfo.n002;

        double fact_w = data.m_f2;
        double fact_u = data.m_f1;
        double fact_v = data.m_f0;

        type::Vector3 normal = n200 * fact_w*fact_w +
                                      n020 * fact_u*fact_u +
                                      n002 * fact_v*fact_v +
                                      tbinfo.n110 * fact_w*fact_u +
                                      tbinfo.n011 * fact_u*fact_v +
                                      tbinfo.n101 * fact_w*fact_v;

        return normal.normalized();
    }

    ////Bezier triangle are computed according to :
    ////http://www.gamasutra.com/view/feature/131389/b%C3%A9zier_triangles_and_npatches.php?print=1
    inline type::Vector3 getPosition(const PROXIMITYDATA & data, core::VecCoordId v = core::VecCoordId::position()) const {
        const BezierTriangleInfo & tbinfo = getBezierInfo(v)[data.m_eid];

        const helper::ReadAccessor<DataVecCoord> & x = this->getState()->read(v);

        const type::Vector3 & p300 = x[data.m_p2];
        const type::Vector3 & p030 = x[data.m_p1];
        const type::Vector3 & p003 = x[data.m_p0];

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
    }


    PROXIMITYDATA createProximity(Index eid, CONTROL_POINT pid = CONTROL_DEFAULT) const {
        return PROXIMITYDATA::create(eid, getTriangle(eid), pid);
    }

    const std::vector<BezierTriangleInfo> & getBezierInfo(core::VecCoordId v = core::VecCoordId::position()) const {
        if (m_bezier_info[v.getIndex()].empty()) recomputeBezierInfo(v);
        return m_bezier_info[v.getIndex()];
    }

protected:
    mutable std::map<int,std::vector<BezierTriangleInfo>> m_bezier_info;

    void recomputeBezierInfo(core::VecCoordId v = core::VecCoordId::position()) const {
        std::vector<BezierTriangleInfo> & vecInfo = m_bezier_info[v.getIndex()];

        const sofa::type::vector<Triangle>& triangles = this->l_topology->getTriangles();
        const helper::ReadAccessor<DataVecCoord> & x = this->getState()->read(v);

        vecInfo.clear();
        for (size_t t=0;t< triangles.size();t++)
        {
            const Triangle& triangle = this->l_topology->getTriangle(t);

            type::Vector3 n200;
            type::Vector3 n020;
            type::Vector3 n002;

            std::cerr << "recomputeBezierInfo needs to compute point normals" << std::endl;
            const sofa::core::topology::BaseMeshTopology::TrianglesAroundVertex & tav0 = this->l_topology->getTrianglesAroundVertex(triangle[2]);
//            for (size_t t=0;t<tav0.size();t++) n200 += this->m_triangle_info[tav0[t]].n;

            const sofa::core::topology::BaseMeshTopology::TrianglesAroundVertex & tav1 = this->l_topology->getTrianglesAroundVertex(triangle[1]);
//            for (size_t t=0;t<tav1.size();t++) n020 += this->m_triangle_info[tav1[t]].n;

            const sofa::core::topology::BaseMeshTopology::TrianglesAroundVertex & tav2 = this->l_topology->getTrianglesAroundVertex(triangle[0]);
//            for (size_t t=0;t<tav2.size();t++) n002 += this->m_triangle_info[tav2[t]].n;

            n200.normalize();
            n020.normalize();
            n002.normalize();

            BezierTriangleInfo tbinfo;
            const Triangle& trpids = triangles[t];

            tbinfo.p300 = x[trpids[2]];
            tbinfo.p030 = x[trpids[1]];
            tbinfo.p003 = x[trpids[0]];

            double w12 = dot(tbinfo.p030 - tbinfo.p300,n200);
            double w21 = dot(tbinfo.p300 - tbinfo.p030,n020);
            double w23 = dot(tbinfo.p003 - tbinfo.p030,n020);
            double w32 = dot(tbinfo.p030 - tbinfo.p003,n002);
            double w31 = dot(tbinfo.p300 - tbinfo.p003,n002);
            double w13 = dot(tbinfo.p003 - tbinfo.p300,n200);

            tbinfo.p210 = (tbinfo.p300*2.0 + tbinfo.p030 - n200 * w12) / 3.0;
            tbinfo.p120 = (tbinfo.p030*2.0 + tbinfo.p300 - n020 * w21) / 3.0;

            tbinfo.p021 = (tbinfo.p030*2.0 + tbinfo.p003 - n020 * w23) / 3.0;
            tbinfo.p012 = (tbinfo.p003*2.0 + tbinfo.p030 - n002 * w32) / 3.0;

            tbinfo.p102 = (tbinfo.p003*2.0 + tbinfo.p300 - n002 * w31) / 3.0;
            tbinfo.p201 = (tbinfo.p300*2.0 + tbinfo.p003 - n200 * w13) / 3.0;

            type::Vector3 E = (tbinfo.p210+tbinfo.p120+tbinfo.p102+tbinfo.p201+tbinfo.p021+tbinfo.p012) / 6.0;
            type::Vector3 V = (tbinfo.p300+tbinfo.p030+tbinfo.p003) / 3.0;
            tbinfo.p111 =  E + (E-V) / 2.0;

            //Compute Bezier Normals
            double v12 = 2 * dot(tbinfo.p030-tbinfo.p300,n200+n020) / dot(tbinfo.p030-tbinfo.p300,tbinfo.p030-tbinfo.p300);
            double v23 = 2 * dot(tbinfo.p003-tbinfo.p030,n020+n002) / dot(tbinfo.p003-tbinfo.p030,tbinfo.p003-tbinfo.p030);
            double v31 = 2 * dot(tbinfo.p300-tbinfo.p003,n002+n200) / dot(tbinfo.p300-tbinfo.p003,tbinfo.p300-tbinfo.p003);

            type::Vector3 h110 = n200 + n020 - (tbinfo.p030-tbinfo.p300) * v12;
            type::Vector3 h011 = n020 + n002 - (tbinfo.p003-tbinfo.p030) * v23;
            type::Vector3 h101 = n002 + n200 - (tbinfo.p300-tbinfo.p003) * v31;

            tbinfo.n110 = h110 / h110.norm();
            tbinfo.n011 = h011 / h011.norm();
            tbinfo.n101 = h101 / h101.norm();

            tbinfo.n200 = n200;
            tbinfo.n020 = n020;
            tbinfo.n002 = n002;

            vecInfo.push_back(tbinfo);
        }
    }

};

}

}
