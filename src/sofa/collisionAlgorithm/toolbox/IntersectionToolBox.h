#pragma once

//#include <sofa/collisionAlgorithm/BaseProximity.h>
#include <sofa/collisionAlgorithm/BaseElement.h>
#include <sofa/collisionAlgorithm/elements/PointElement.h>
#include <sofa/collisionAlgorithm/elements/EdgeElement.h>
#include <sofa/collisionAlgorithm/elements/TriangleElement.h>
#include <sofa/collisionAlgorithm/elements/TetrahedronElement.h>
#include <sofa/collisionAlgorithm/proximity/PointProximity.h>
#include <sofa/collisionAlgorithm/toolbox/EdgeToolBox.h>
// #include <sofa/component/topology/container/dynamic/EdgeSetTopologyModifier.h>
//#include <EdgeSetTopologyModifier.h>
//#include <EdgeSetGeometryAlgorithm.h>

namespace sofa::collisionAlgorithm::toolbox {

class IntersectionToolBox {
public:


    static BaseElement::SPtr intersect_edge_tetra(BaseElement* e1, BaseElement* e2) {
//        auto edge = e1->element_cast<EdgeElement>();
//        auto tetra = e2->element_cast<TetrahedronElement>();


//        return point->createProximity();
    }

    static BaseElement::SPtr intersect_tetra_edge(BaseElement* e1, BaseElement* e2) {
        intersect_edge_tetra(e2,e1);
    }

  // /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Based on https://en.wikipedia.org/wiki/Line%E2%80%93plane_intersection
    static BaseElement::SPtr intersect_edge_triangle(BaseElement* e1, BaseElement* e2) {
        auto edge = e1->element_cast<EdgeElement>();
        auto triangle = e2->element_cast<TriangleElement>();

        // Compute the Plucker coefficient for the edges (--> in edgeToolbox ?)
        double pluckerT0[6];
        double pluckerT1[6];
        double pluckerT2[6];
        double pluckerEdge[6];

        compute_plucker_coeffs(pluckerT0, triangle->getP0(), triangle->getP1());
        compute_plucker_coeffs(pluckerT1, triangle->getP1(), triangle->getP2());
        compute_plucker_coeffs(pluckerT2, triangle->getP2(), triangle->getP0());
        compute_plucker_coeffs(pluckerEdge, edge->getP0(), edge->getP1());


        // Compute the side product related to the three edges of the triangle
        double s0 = side_product(pluckerEdge,pluckerT0);
        double s1 = side_product(pluckerEdge,pluckerT1);
        double s2 = side_product(pluckerEdge,pluckerT2);

        // Algorithm to compute the intersection --> calls the algo for 2D interection


        // Other approach
//        sofa::type::Vector3 line =  edge->getP1()->getPosition() - edge->getP0()->getPosition();
//        sofa::type::Vector3 T01 = triangle->getP1()->getPosition() - triangle->getP0()->getPosition();
//        sofa::type::Vector3 T02 = triangle->getP2()->getPosition() - triangle->getP0()->getPosition();
//        double determinant = -dot(line, T01.cross(T02));



//        return point->createProximity();
    }

    static BaseElement::SPtr intersect_triangle_edge(BaseElement* e1, BaseElement* e2) {
        intersect_edge_triangle(e2,e1);
    }





    static BaseElement::SPtr intersect_edge_point(BaseElement* e1, BaseElement* e2) {
//        auto edge = e1->element_cast<EdgeElement>();
//        auto point = e2->element_cast<PointElement>();

//        return /// ;
    }


    static BaseElement::SPtr intersect_point_edge(BaseElement* e1, BaseElement* e2) {
        intersect_edge_point(e2,e1);
    }




    static BaseElement::SPtr intersect_edge_edge(BaseElement* e1, BaseElement* e2) {
//        auto edge1 = e1->element_cast<EdgeElement>();
//        auto edge2 = e2->element_cast<EdgeElement>();


        // Compute intersection coordinates

        // find barycentric coord of that point in the edge

        // Create proximity from those bary coord



//        return ///;
    }




// The following functions are based on:
//     https://members.loria.fr/SLazard/ARC-Visi3D/Pant-project/files/Line_Triangle.html

    static void compute_plucker_coeffs(double plucker[6], BaseProximity::SPtr p, BaseProximity::SPtr q)
    {
        sofa::type::Vector3 p1 = p->getPosition();
        sofa::type::Vector3 q1 = q->getPosition();

        plucker[0]= p1[0]*q1[1]-q1[0]*p1[1];
        plucker[1]= p1[0]*q1[2]-q1[0]*p1[2];
        plucker[2]= p1[0]-q1[0];
        plucker[3]= p1[1]*q1[2]-q1[1]*p1[2];
        plucker[4]= p1[2]-q1[2];
        plucker[5]= q1[1]-p1[1];
    }

    static double side_product(double * l1, double * l2)
    {
        double result = 0;

        result = l1[0]*l2[4] + l1[1]*l2[5] + l1[2]*l2[3] + l1[3]*l2[2] + l1[4]*l2[0] + l1[5]*l2[1];
        return result;
    }


    static void compute_2D_intersection(TriangleElement* T, EdgeElement* L) {
//        std::vector<PointElement> p[4];	// Four trial points
//        PointElement x;	// The selected point : This point does not lie in the same plane
//        TriangleElement T1;	// Trial triangle constructed to test coplanarity
//        int edge[] = {0,0,0},
//            vertex[] ={0,0,0}; 	// Markers for intersection

//        int i,intersection=0;
//        double s1,s2,s3;

    }




};



}

