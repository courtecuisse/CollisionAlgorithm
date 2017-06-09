#ifndef SOFA_COMPONENT_CONSTRAINT_FINDCLOSESTALGO_H
#define SOFA_COMPONENT_CONSTRAINT_FINDCLOSESTALGO_H

#include "CollisionAlgorithm.h"
#include "ConstraintProximity.h"
#include <sofa/defaulttype/SolidTypes.h>
#include <sofa/core/behavior/BaseController.h>
#include <sofa/core/behavior/MechanicalState.h>
#include <math.h>
#include <sofa/defaulttype/Vec.h>
#include "AABBDecorator.h"
#include <sofa/core/ObjectFactory.h>
#include <sofa/core/visual/VisualParams.h>
#include <SofaOpenglVisual/OglModel.h>
#include <math.h>
#include <assert.h>     /* assert */
#include "TriangleNonLinearGeometry.h"

namespace sofa {

namespace core {

namespace behavior {

ConstraintProximityPtr CollisionAlgorithm::newtonTriangularIterations(const defaulttype::Vector3 & P,unsigned eid,const ConstraintProximityPtr & pfrom,unsigned max_it, double tolerance, double threshold) {
    unsigned int it=0;
    double delta = 0.00001;

    const TriangleNonLinearGeometry * geo = dynamic_cast<const TriangleNonLinearGeometry *>(pfrom->m_cg);
    if (geo == NULL) {
        std::cerr << "Error : CollisionAlgorithm::newtonTriangularIterations must be used with a TriangleNonLinearGeometry" << std::endl;
        return NULL;
    }

    int p0 = pfrom->m_pid[0];
    int p1 = pfrom->m_pid[1];
    int p2 = pfrom->m_pid[2];

    ConstraintProximityPtr pinfo = geo->getNonLinearTriangleProximity(eid,p0,pfrom->m_fact[0],p1,pfrom->m_fact[1],p2,pfrom->m_fact[2]);

    while(it< max_it) {
        defaulttype::Vector3 Q = pinfo->getPosition();

        defaulttype::Vector3 nQP = P - Q;
        if (nQP.norm() < tolerance) break;
        nQP.normalize();

        defaulttype::Vector3 N1 = pinfo->getNormal();
        N1.normalize();

        if (pinfo->m_fact[0] < 0 || pinfo->m_fact[1] < 0 || pinfo->m_fact[2] < 0) break;

        defaulttype::Vector3 N2 = cross(N1,((fabs(dot(N1,defaulttype::Vector3(1,0,0)))>0.99) ? defaulttype::Vector3(0,1,0) : defaulttype::Vector3(1,0,0)));
        N2.normalize();

        defaulttype::Vector3 N3 = cross(N1,N2);
        N3.normalize();

        defaulttype::Vector2 e_0(dot(nQP,N2),dot(nQP,N3));

        if(e_0.norm() < tolerance) break;

        double fact_u = (pinfo->m_fact[2] - delta < 0.0 || pinfo->m_fact[1] + delta > 1.0) ? -1.0 : 1.0;
        double fact_v = (pinfo->m_fact[2] - delta < 0.0 || pinfo->m_fact[0] + delta > 1.0) ? -1.0 : 1.0;

        //variation point along v
        double P_v_fact0 = pinfo->m_fact[0] + delta * fact_v;
        double P_v_fact1 = pinfo->m_fact[1];
        double P_v_fact2 = pinfo->m_fact[2] - delta * fact_v;
        ConstraintProximityPtr P_v = geo->getNonLinearTriangleProximity(eid,p0,P_v_fact0,p1,P_v_fact1,p2,P_v_fact2);
        defaulttype::Vector3 p_v = (P - P_v->getPosition()).normalized();
        defaulttype::Vector2 e_v(dot(p_v,N2)*fact_v,dot(p_v,N3)*fact_v);

        //variation point along u
        double P_u_fact0 = pinfo->m_fact[0];
        double P_u_fact1 = pinfo->m_fact[1] + delta * fact_u;
        double P_u_fact2 = pinfo->m_fact[2] - delta * fact_u;
        ConstraintProximityPtr P_u = geo->getNonLinearTriangleProximity(eid,p0,P_u_fact0,p1,P_u_fact1,p2,P_u_fact2);
        defaulttype::Vector3 p_u = (P - P_u->getPosition()).normalized();
        defaulttype::Vector2 e_u(dot(p_u,N2)*fact_u,dot(p_u,N3)*fact_u);

        if (P_v->m_fact[0] < 0 || P_v->m_fact[1] < 0 || P_v->m_fact[2] < 0) break;
        if (P_u->m_fact[0] < 0 || P_u->m_fact[1] < 0 || P_u->m_fact[2] < 0) break;

        defaulttype::Mat<2,2,double> J, invJ;
        J[0][0] = (e_v[0] - e_0[0])/delta;
        J[1][0] = (e_v[1] - e_0[1])/delta;
        J[0][1] = (e_u[0] - e_0[0])/delta;
        J[1][1] = (e_u[1] - e_0[1])/delta;

        invertMatrix(invJ, J);

        // dUV is the optimal direction
        defaulttype::Vector2 dUV = -invJ * e_0;
        if(dUV.norm() < threshold) break;

        //bary coords of the solution of the 2D problem
        double sol_v = pinfo->m_fact[0] + dUV[0];
        double sol_u = pinfo->m_fact[1] + dUV[1];
        double sol_w = 1.0 - sol_u - sol_v;

        // we now search what is the optimal displacmeent along this path
        defaulttype::Vector3 dir2d(sol_v - pinfo->m_fact[0],
                                   sol_u - pinfo->m_fact[1],
                                   sol_w - pinfo->m_fact[2]);

        if(dir2d.norm() < threshold) break;

        //we apply a small perturbation arond the 2d direction
        dir2d.normalize();

        double fact_a = (pinfo->m_fact[0] + dir2d[0] * delta < 0 || pinfo->m_fact[1] + dir2d[1] * delta < 0 || pinfo->m_fact[2] + dir2d[2] * delta < 0) ? -1.0 : 1.0;
        double P_a_fact0 = pinfo->m_fact[0] + dir2d[0] * delta * fact_a;
        double P_a_fact1 = pinfo->m_fact[1] + dir2d[1] * delta * fact_a;
        double P_a_fact2 = pinfo->m_fact[2] + dir2d[2] * delta * fact_a;
        ConstraintProximityPtr P_a = geo->getNonLinearTriangleProximity(eid,p0,P_a_fact0,p1,P_a_fact1,p2,P_a_fact2);
        if (P_a->m_fact[0] < 0 || P_a->m_fact[1] < 0 || P_a->m_fact[2] < 0) break;
        defaulttype::Vector3 QA = P_a->getPosition();

        double fact;
        if (fabs(dot(nQP,N1))>0.8) {
            double fx = acos(fabs(dot(nQP,N1)));
            double fxdx = acos(fabs(dot((P - QA).normalized(),P_a->getNormal())));
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

        double new_v = pinfo->m_fact[0] + dir2d[0];
        double new_u = pinfo->m_fact[1] + dir2d[1];
        double new_w = pinfo->m_fact[2] + dir2d[2];

        if (new_v<0 && fabs(dir2d[0])>0) dir2d *= -pinfo->m_fact[0] / dir2d[0];
        if (new_u<0 && fabs(dir2d[1])>0) dir2d *= -pinfo->m_fact[1] / dir2d[1];
        if (new_w<0 && fabs(dir2d[2])>0) dir2d *= -pinfo->m_fact[2] / dir2d[2];

        pinfo->m_fact[0] += dir2d[0];
        pinfo->m_fact[1] += dir2d[1];
        pinfo->m_fact[2] += dir2d[2];

        it++;
    }

    return pinfo;
}

} // namespace controller

} // namespace component

} // namespace sofa

#endif // SOFA_COMPONENT_CONTROLLER_NeedleConstraint_H
