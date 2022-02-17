#pragma once

#include <sofa/collisionAlgorithm/BaseProximity.h>
#include <sofa/collisionAlgorithm/toolbox/TriangleToolBox.h>
#include <sofa/collisionAlgorithm/elements/TetrahedronElement.h>

namespace sofa::collisionAlgorithm::toolbox {

class TetrahedronToolBox {
public:

    static BaseProximity::SPtr createCenterProximity(BaseElement::SPtr elmt) {
        TetrahedronElement * tetra = elmt->element_cast<TetrahedronElement>();
        return tetra->createProximity(1.0/4.0,1.0/4.0,1.0/4.0,1.0/4.0);
    }

    //Barycentric coordinates are computed according to
    //http://gamedev.stackexchange.com/questions/23743/whats-the-most-efficient-way-to-find-barycentric-coordinates
    static BaseProximity::SPtr project(type::Vector3 P, BaseElement::SPtr elmt) {
        TetrahedronElement * tetra = elmt->element_cast<TetrahedronElement>();

        double fact[4];
        projectOnTetra(P, tetra,fact[0],fact[1],fact[2],fact[3]);
        return tetra->createProximity(fact[0],fact[1],fact[2],fact[3]);
    }


    static void projectOnTetra(const type::Vec3d projectP, TetrahedronElement * tetra, double & fact_u, double & fact_v, double & fact_w,double & fact_x) {
        const TetrahedronElement::TetraInfo & tinfo = tetra->getTetrahedronInfo();

        computeTetraBaryCoords(projectP, tinfo, fact_u,fact_v,fact_w,fact_x);

//        if(fact_u<0) {
//            fact_u = 0;
//            projectOnTriangle(projectP, computeTriangleInfo(tinfo.P1, tinfo.P2, tinfo.P3), fact_v,fact_w,fact_x);
//        } else if(fact_v<0) {
//            fact_v = 0;
//            projectOnTriangle(projectP, computeTriangleInfo(tinfo.P0, tinfo.P2, tinfo.P3),fact_u,fact_w,fact_x);
//        } else if(fact_w<0) {
//            fact_w = 0;
//            projectOnTriangle(projectP, computeTriangleInfo(tinfo.P0, tinfo.P1, tinfo.P3),fact_u,fact_v,fact_x);
//        } else if(fact_x<0) {
//            fact_x = 0;
//            projectOnTriangle(projectP, computeTriangleInfo(tinfo.P0, tinfo.P1, tinfo.P2),fact_u,fact_v,fact_w);
//        }
    }

    static void computeTetraBaryCoords(const type::Vec3d & P, const TetrahedronElement::TetraInfo & tinfo, double & fact_u,double & fact_v, double & fact_w, double & fact_x) {
        type::Vec3d e = P - tinfo.P0;

        double Va = 1.0/6.0 * dot(e,tinfo.ax2Cax3);
        double Vb = 1.0/6.0 * dot(tinfo.ax1,e.cross(tinfo.ax3));
        double Vc = 1.0/6.0 * dot(tinfo.ax1,tinfo.ax2.cross(e));

        fact_v = Va/tinfo.V0;
        fact_w = Vb/tinfo.V0;
        fact_x = Vc/tinfo.V0;
        fact_u = 1.0 - (fact_v + fact_w + fact_x);
    }

};



}

