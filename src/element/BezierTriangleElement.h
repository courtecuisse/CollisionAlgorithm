﻿#pragma once

#include <geometry/BezierTriangleGeometry.h>
#include <element/TriangleElement.h>

namespace collisionAlgorithm {

/**************************************************************************/
/******************************ELEMENT*************************************/
/**************************************************************************/

class BezierTriangleElement : public ConstraintElement {
public:

    ConstraintProximityPtr createProximity(double f1,double f2,double f3);

    BezierTriangleElement(BezierTriangleGeometry * geo,unsigned eid) : ConstraintElement(geo,3) {
        m_eid = eid;

        const std::vector<Topology::Triangle> & triangles = geometry()->p_topology->getTriangles();

        m_pid[0] = triangles[eid][0];
        m_pid[1] = triangles[eid][1];
        m_pid[2] = triangles[eid][2];
    }

    ConstraintProximityPtr getControlPoint(const int cid) {
        if (cid == 0) return createProximity(1,0,0);
        else if (cid == 1) return createProximity(0,1,0);
        else if (cid == 2) return createProximity(0,0,1);
        return createProximity(1.0/3.0,1.0/3.0,1.0/3.0);
    }

    ConstraintProximityPtr project(Vector3 P) {
        //initialize the algorithm xith the projection on a linear triangle
        std::map<unsigned,double> contribs = TriangleElement(geometry(),m_eid).project(P)->getContributions();

        double fact[3];
        fact[0] = contribs[m_pid[0]];
        fact[1] = contribs[m_pid[1]];
        fact[2] = contribs[m_pid[2]];
    //    ConstraintProximityPtr((ConstraintProximity *)pfrom)->refineToClosestPoint(P);

        unsigned max_it = geometry()->d_nonlin_max_it.getValue();
        double tolerance = geometry()->d_nonlin_tolerance.getValue();
        double threshold = geometry()->d_nonlin_threshold.getValue();

        unsigned int it=0;
        double delta = 0.00001;

        BezierConstraintProximity pinfo(this,pfrom->m_eid,pfrom->m_pid[0],pfrom->m_fact[0],pfrom->m_pid[1],pfrom->m_fact[1],pfrom->m_pid[2],pfrom->m_fact[2]);

        while(it< max_it) {
            Vector3 Q = pinfo.getPosition();

            Vector3 nQP = P - Q;
            if (nQP.norm() < tolerance) break;
            nQP.normalize();

            Vector3 N1 = pinfo.getNormal();
            N1.normalize();

            if (pinfo.m_fact[0] < 0 || pinfo.m_fact[1] < 0 || pinfo.m_fact[2] < 0) break;

            Vector3 N2 = cross(N1,((fabs(dot(N1,Vector3(1,0,0)))>0.99) ? Vector3(0,1,0) : Vector3(1,0,0)));
            N2.normalize();

            Vector3 N3 = cross(N1,N2);
            N3.normalize();

            Vector2 e_0(dot(nQP,N2),dot(nQP,N3));

            if(e_0.norm() < tolerance) break;

            double fact_u = (pinfo.m_fact[2] - delta < 0.0 || pinfo.m_fact[1] + delta > 1.0) ? -1.0 : 1.0;
            double fact_v = (pinfo.m_fact[2] - delta < 0.0 || pinfo.m_fact[0] + delta > 1.0) ? -1.0 : 1.0;

            //variation point along v
            double P_v_fact0 = pinfo.m_fact[0] + delta * fact_v;
            double P_v_fact1 = pinfo.m_fact[1];
            double P_v_fact2 = pinfo.m_fact[2] - delta * fact_v;

            BezierConstraintProximity P_v(this,pinfo.m_eid,p0,P_v_fact0,p1,P_v_fact1,p2,P_v_fact2);
            Vector3 p_v = (P - P_v.getPosition()).normalized();
            Vector2 e_v(dot(p_v,N2)*fact_v,dot(p_v,N3)*fact_v);

            //variation point along u
            double P_u_fact0 = pinfo.m_fact[0];
            double P_u_fact1 = pinfo.m_fact[1] + delta * fact_u;
            double P_u_fact2 = pinfo.m_fact[2] - delta * fact_u;
            BezierConstraintProximity P_u(this,pinfo.m_eid,p0,P_u_fact0,p1,P_u_fact1,p2,P_u_fact2);
            Vector3 p_u = (P - P_u.getPosition()).normalized();
            Vector2 e_u(dot(p_u,N2)*fact_u,dot(p_u,N3)*fact_u);

            if (P_v.m_fact[0] < 0 || P_v.m_fact[1] < 0 || P_v.m_fact[2] < 0) break;
            if (P_u.m_fact[0] < 0 || P_u.m_fact[1] < 0 || P_u.m_fact[2] < 0) break;

            Mat<2,2,double> J, invJ;
            J[0][0] = (e_v[0] - e_0[0])/delta;
            J[1][0] = (e_v[1] - e_0[1])/delta;
            J[0][1] = (e_u[0] - e_0[0])/delta;
            J[1][1] = (e_u[1] - e_0[1])/delta;

            invertMatrix(invJ, J);

            // dUV is the optimal direction
            Vector2 dUV = -invJ * e_0;
            if(dUV.norm() < threshold) break;

            //bary coords of the solution of the 2D problem
            double sol_v = pinfo.m_fact[0] + dUV[0];
            double sol_u = pinfo.m_fact[1] + dUV[1];
            double sol_w = 1.0 - sol_u - sol_v;

            // we now search what is the optimal displacmeent along this path
            Vector3 dir2d(sol_v - pinfo.m_fact[0],
                                       sol_u - pinfo.m_fact[1],
                                       sol_w - pinfo.m_fact[2]);

            if(dir2d.norm() < threshold) break;

            //we apply a small perturbation arond the 2d direction
            dir2d.normalize();

            double fact_a = (pinfo.m_fact[0] + dir2d[0] * delta < 0 || pinfo.m_fact[1] + dir2d[1] * delta < 0 || pinfo.m_fact[2] + dir2d[2] * delta < 0) ? -1.0 : 1.0;
            double P_a_fact0 = pinfo.m_fact[0] + dir2d[0] * delta * fact_a;
            double P_a_fact1 = pinfo.m_fact[1] + dir2d[1] * delta * fact_a;
            double P_a_fact2 = pinfo.m_fact[2] + dir2d[2] * delta * fact_a;
            BezierConstraintProximity P_a(this,pinfo.m_eid,p0,P_a_fact0,p1,P_a_fact1,p2,P_a_fact2);

            if (P_a.m_fact[0] < 0 || P_a.m_fact[1] < 0 || P_a.m_fact[2] < 0) break;

            Vector3 QA = P_a.getPosition();

            double fact;
            if (fabs(dot(nQP,N1))>0.8) {
                double fx = acos(fabs(dot(nQP,N1)));
                double fxdx = acos(fabs(dot((P - QA).normalized(),P_a.getNormal())));
                double j = (fxdx - fx) / delta;
                fact = -fx / j;
            } else {
                Vector3 nQA = (Q-QA).normalized();
                double fx = dot(P-Q, nQA);
                double fxdx = dot(P-QA, nQA);
                double j = (fxdx - fx) / delta;
                fact = -fx / j;
            }

            if(fabs(fact) < threshold) break;

            dir2d *= fact * fact_a;

            double new_v = pinfo.m_fact[0] + dir2d[0];
            double new_u = pinfo.m_fact[1] + dir2d[1];
            double new_w = pinfo.m_fact[2] + dir2d[2];

            if (new_v<0 && fabs(dir2d[0])>0) dir2d *= -pinfo.m_fact[0] / dir2d[0];
            if (new_u<0 && fabs(dir2d[1])>0) dir2d *= -pinfo.m_fact[1] / dir2d[1];
            if (new_w<0 && fabs(dir2d[2])>0) dir2d *= -pinfo.m_fact[2] / dir2d[2];

            pinfo.m_fact[0] += dir2d[0];
            pinfo.m_fact[1] += dir2d[1];
            pinfo.m_fact[2] += dir2d[2];

            it++;
        }

        pfrom->m_fact[0] = pinfo.m_fact[0];
        pfrom->m_fact[1] = pinfo.m_fact[1];
        pfrom->m_fact[2] = pinfo.m_fact[2];
    }

    inline BezierTriangleGeometry * geometry() const {
        return (BezierTriangleGeometry*) m_geometry;
    }

protected:
    Vector3 m_pos;
    unsigned m_pid[3];
    unsigned m_eid;

};

/**************************************************************************/
/******************************PROXIMITY***********************************/
/**************************************************************************/

class BezierTriangleProximity : public ConstraintProximity {
public:
    friend class BezierTriangleGeometry;

    BezierTriangleProximity(const BezierTriangleGeometry * geo, unsigned eid, unsigned p1,double f1,unsigned p2, double f2, unsigned p3, double f3) : ConstraintProximity(geo) {
        m_eid = eid;

        m_pid.resize(3);
        m_fact.resize(3);

        m_pid[0] = p1;
        m_fact[0] = f1;

        m_pid[1] = p2;
        m_fact[1] = f2;

        m_pid[2] = p3;
        m_fact[2] = f3;
    }

    ////Bezier triangle are computed according to :
    ////http://www.gamasutra.com/view/feature/131389/b%C3%A9zier_triangles_and_npatches.php?print=1
    Vector3 getPosition() const {
        const BezierTriangleInfo & tbinfo = ((BezierTriangleGeometry *)m_geo)->m_beziertriangle_info[m_eid];
        const helper::ReadAccessor<Data <VecCoord> > & x = ((BezierTriangleGeometry *)m_geo)->getMstate()->read(core::VecCoordId::position());

        const Vector3 & p300 = x[m_pid[2]];
        const Vector3 & p030 = x[m_pid[1]];
        const Vector3 & p003 = x[m_pid[0]];

        double fact_w = m_fact[2];
        double fact_u = m_fact[1];
        double fact_v = m_fact[0];

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

    Vector3 getFreePosition() const {
        double fact_w = m_fact[2];
        double fact_u = m_fact[1];
        double fact_v = m_fact[0];

        const ReadAccessor<Vector3> & x = ((BezierTriangleGeometry *)m_geo)->getMstate()->read(core::VecCoordId::freePosition());

        const Vector3 & p300_Free = x[m_pid[2]];
        const Vector3 & p030_Free = x[m_pid[1]];
        const Vector3 & p003_Free = x[m_pid[0]];

        const Vector3 & n200_Free = ((BezierTriangleGeometry *)m_geo)->m_pointNormal[m_pid[2]];
        const Vector3 & n020_Free = ((BezierTriangleGeometry *)m_geo)->m_pointNormal[m_pid[1]];
        const Vector3 & n002_Free = ((BezierTriangleGeometry *)m_geo)->m_pointNormal[m_pid[0]];

        double w12_free = dot(p030_Free - p300_Free,n200_Free);
        double w21_free = dot(p300_Free - p030_Free,n020_Free);
        double w23_free = dot(p003_Free - p030_Free,n020_Free);
        double w32_free = dot(p030_Free - p003_Free,n002_Free);
        double w31_free = dot(p300_Free - p003_Free,n002_Free);
        double w13_free = dot(p003_Free - p300_Free,n200_Free);

        const Vector3 & p210_Free = (p300_Free*2.0 + p030_Free - n200_Free * w12_free) / 3.0;
        const Vector3 & p120_Free = (p030_Free*2.0 + p300_Free - n020_Free * w21_free) / 3.0;

        const Vector3 & p021_Free = (p030_Free*2.0 + p003_Free - n020_Free * w23_free) / 3.0;
        const Vector3 & p012_Free = (p003_Free*2.0 + p030_Free - n002_Free * w32_free) / 3.0;

        const Vector3 & p102_Free = (p003_Free*2.0 + p300_Free - n002_Free * w31_free) / 3.0;
        const Vector3 & p201_Free = (p300_Free*2.0 + p003_Free - n200_Free * w13_free) / 3.0;

        const Vector3 & E_Free = (p210_Free+p120_Free+p102_Free+p201_Free+p021_Free+p012_Free) / 6.0;
        const Vector3 & V_Free = (p300_Free+p030_Free+p003_Free) / 3.0;
        const Vector3 & p111_Free =  E_Free + (E_Free-V_Free) / 2.0;

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

    Vector3 getNormal() {
        const BezierTriangleInfo & tbinfo = ((BezierTriangleGeometry *)m_geo)->m_beziertriangle_info[m_eid];

        const Vector3 &n200 = ((BezierTriangleGeometry *)m_geo)->m_pointNormal[m_pid[2]];
        const Vector3 &n020 = ((BezierTriangleGeometry *)m_geo)->m_pointNormal[m_pid[1]];
        const Vector3 &n002 = ((BezierTriangleGeometry *)m_geo)->m_pointNormal[m_pid[0]];

        double fact_w = m_fact[2];
        double fact_u = m_fact[1];
        double fact_v = m_fact[0];

        Vector3 normal = n200 * fact_w*fact_w +
                         n020 * fact_u*fact_u +
                         n002 * fact_v*fact_v +
                         tbinfo.n110 * fact_w*fact_u +
                         tbinfo.n011 * fact_u*fact_v +
                         tbinfo.n101 * fact_w*fact_v;

        Vector3 N1 = normal;
        N1.normalize();

        return N1;
    }

    void refineToClosestPoint(const Vector3 & P) {
        ((const BezierTriangleGeometry *) m_geo)->projectPoint(P,this);
    }

protected:
    unsigned m_eid;
};

ConstraintProximityPtr BezierTriangleElement::createProximity(double f1,double f2,double f3) {
    return std::make_shared<BezierTriangleProximity>(this,f1,f2,f3);
}

}
