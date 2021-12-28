#pragma once

#include <sofa/collisionAlgorithm/BaseProximity.h>
#include <sofa/collisionAlgorithm/BaseOperation.h>
#include <sofa/collisionAlgorithm/operations/PointOperation.h>
#include <sofa/collisionAlgorithm/operations/EdgeOperation.h>

namespace sofa::collisionAlgorithm::Operations {

//static int createPointProximity =

class TriangleOperation : public BaseOperation {
public:

    static BaseProximity::SPtr createCenterProximity(BaseElement::SPtr elmt) {
        auto tri = toTriangleElement(elmt);



        return BaseProximity::SPtr(new TriangleProximity(tri->getP0(),tri->getP1(),tri->getP2(),1.0/3.0,1.0/3.0,1.0/3.0));
    }

    //Barycentric coordinates are computed according to
    //http://gamedev.stackexchange.com/questions/23743/whats-the-most-efficient-way-to-find-barycentric-coordinates
    static BaseProximity::SPtr project(BaseProximity::SPtr P, BaseElement::SPtr elmt) {
        auto tri = toTriangleElement(elmt);

        const TriangleElement::TriangleInfo & tinfo = tri->getTriangleInfo();
//        auto triangle = getTriangle(eid);

        double fact_u,fact_v,fact_w;
        projectOnTriangle(P->getPosition(),tinfo,fact_u,fact_v,fact_w);

        return elmt.createProximity(fact_u,fact_v,fact_w);
//        return BaseProximity::SPtr(new TriangleProximity(tri->getP0(),tri->getP1(),tri->getP2(),fact_u,fact_v,fact_w));
    }


    static void computeTriangleBaryCoords(const type::Vec3d & proj_P, const TriangleElement::TriangleInfo & tinfo, double & fact_u, double & fact_v, double & fact_w)
    {
        type::Vec3d v2 = proj_P - tinfo.P0;

        double d20 = dot(v2,tinfo.v0);
        double d21 = dot(v2,tinfo.v1);

        fact_v = (tinfo.d11 * d20 - tinfo.d01 * d21) * tinfo.invDenom;
        fact_w = (tinfo.d00 * d21 - tinfo.d01 * d20) * tinfo.invDenom;
        fact_u = 1.0 - fact_v  - fact_w;
    }

    static void projectOnTriangle(const type::Vec3d projectP, const TriangleElement::TriangleInfo & tinfo, double & fact_u, double & fact_v, double & fact_w) {
        type::Vec3d x1x2 = projectP - tinfo.P0;

        //corrdinate on the plane
        double c0 = dot(x1x2,tinfo.ax1);
        double c1 = dot(x1x2,tinfo.ax2);
        type::Vec3d proj_P = tinfo.P0 + tinfo.ax1 * c0 + tinfo.ax2 * c1;

        computeTriangleBaryCoords(proj_P, tinfo, fact_u,fact_v,fact_w);

        if (fact_u<0)
        {
            EdgeOperation::projectOnEdge(proj_P, tinfo.P1, tinfo.P2, fact_v, fact_w);
            fact_u=0;
        }
        else if (fact_v<0)
        {
            EdgeOperation::projectOnEdge(proj_P, tinfo.P0, tinfo.P2, fact_u, fact_w);
            fact_v=0;
        }
        else if (fact_w<0)
        {
            EdgeOperation::projectOnEdge(proj_P, tinfo.P0, tinfo.P1, fact_u, fact_v);
            fact_w=0;
        }
    }

protected:
    TriangleOperation() {
        static TriangleOperation s_triop;

        CreateCenterProximity::register_func(&s_triop,&TriangleOperation::createCenterProximity);

        Project::register_func(&s_triop,&TriangleOperation::project);
    }

    static const TriangleElement * toTriangleElement(BaseElement::SPtr elmt) {
        return (TriangleElement *) elmt.get();
    }

};



}

