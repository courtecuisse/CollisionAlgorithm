#pragma once

#include <sofa/collisionAlgorithm/BaseProximity.h>
#include <sofa/collisionAlgorithm/toolbox/EdgeToolBox.h>
#include <sofa/collisionAlgorithm/elements/TriangleElement.h>

namespace sofa::collisionAlgorithm::toolbox {

class TriangleToolBox {
public:

    static BaseProximity::SPtr createCenterProximity(BaseElement* elmt) {
        TriangleElement * tri = elmt->element_cast<TriangleElement>();
        return tri->createProximity(1.0/3.0,1.0/3.0,1.0/3.0);
    }

    //Barycentric coordinates are computed according to
    //http://gamedev.stackexchange.com/questions/23743/whats-the-most-efficient-way-to-find-barycentric-coordinates
    static BaseProximity::SPtr project(type::Vector3 P, BaseElement* elmt) {
        TriangleElement * tri = elmt->element_cast<TriangleElement>();

        double fact_u,fact_v,fact_w;
        projectOnTriangle(P,tri->getTriangleInfo(),fact_u,fact_v,fact_w);
//        std::cout << "test coord bary après : " << fact_u << "  ||  " << fact_v << "  ||  " << fact_w << std::endl;
        return tri->createProximity(fact_u,fact_v,fact_w);
    }


    static void computeTriangleBaryCoords(const type::Vec3d & proj_P, const TriangleElement::TriangleInfo & tinfo, double & fact_u, double & fact_v, double & fact_w)
    {
//        std::cout << "test tinfo1 : " << tinfo.P0 << "  ||  " << tinfo.v0 << "  ||  " << tinfo.v1 << std::endl;
//        std::cout << "test tinfo1 : " << tinfo.d11 << "  ||  " << tinfo.d01 << "  ||  " << tinfo.d00 << "  ||  " << tinfo.invDenom << std::endl;
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

        if (fact_u<0) {
            EdgeToolBox::projectOnEdge(proj_P, tinfo.P1, tinfo.P2, fact_v, fact_w);
            fact_u=0;
        } else if (fact_v<0) {
            EdgeToolBox::projectOnEdge(proj_P, tinfo.P0, tinfo.P2, fact_u, fact_w);
            fact_v=0;
        } else if (fact_w<0) {
            EdgeToolBox::projectOnEdge(proj_P, tinfo.P0, tinfo.P1, fact_u, fact_v);
            fact_w=0;
        }
    }

};



}

