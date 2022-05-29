#pragma once

//#include <sofa/collisionAlgorithm/BaseProximity.h>
#include <sofa/collisionAlgorithm/BaseElement.h>
#include <sofa/collisionAlgorithm/elements/PointElement.h>
#include <sofa/collisionAlgorithm/elements/EdgeElement.h>
#include <sofa/collisionAlgorithm/elements/TriangleElement.h>
#include <sofa/collisionAlgorithm/elements/TetrahedronElement.h>
#include <sofa/collisionAlgorithm/proximity/PointProximity.h>
#include <sofa/collisionAlgorithm/proximity/EdgeProximity.h>
#include <sofa/collisionAlgorithm/toolbox/EdgeToolBox.h>
#include <sofa/collisionAlgorithm/toolbox/TetrahedronToolBox.h>
#include <sofa/type/Mat.h>
// #include <sofa/component/topology/container/dynamic/EdgeSetTopologyModifier.h>
//#include <EdgeSetTopologyModifier.h>
//#include <EdgeSetGeometryAlgorithm.h>

namespace sofa::collisionAlgorithm::toolbox {

class IntersectionToolBox {
public:


    static std::pair<BaseElement::SPtr,BaseElement::SPtr> intersect_edge_tetra(BaseElement::SPtr e1, BaseElement::SPtr e2) { /*std::cout << "intersect_edge_tetra" << std::endl;*/
        auto edge = e1->edgeElements()[0];
        auto tetra = e2->tetrahedronElements()[0];

        type::Vector3 eP0 = edge->getP0()->getPosition();
        type::Vector3 eP1 = edge->getP1()->getPosition();
        TetrahedronElement::TetraInfo tetraInfo = tetra->getTetrahedronInfo();
        double factE0[4];
        double factE1[4];


        // If the edge is entirely included in the tetrahedron : the intersection is the edge itself
        if (toolbox::TetrahedronToolBox::isInTetra(eP0, tetraInfo, factE0[0], factE0[1], factE0[2], factE0[3])
         && toolbox::TetrahedronToolBox::isInTetra(eP1, tetraInfo, factE1[0], factE1[1], factE1[2], factE1[3])) {
//            std::cout << "original edge returned" << std::endl;
            EdgeElement::SPtr edge = EdgeElement::create(edge->getP0(), edge->getP1());

            auto proxTetra1 = TetrahedronToolBox::project(edge->getP0()->getPosition(), tetra);
            auto proxTetra2 = TetrahedronToolBox::project(edge->getP1()->getPosition(), tetra);
            EdgeElement::SPtr edgeFromTetra = EdgeElement::create(proxTetra1, proxTetra2);

            return std::pair<BaseElement::SPtr,BaseElement::SPtr>(edge,edgeFromTetra);
        }


        auto proxP0 = tetra->getP0();
        auto proxP1 = tetra->getP1();
        auto proxP2 = tetra->getP2();
        auto proxP3 = tetra->getP3();

        auto  triangle0 = TriangleElement::create(proxP0, proxP1, proxP2);
        auto  triangle1 = TriangleElement::create(proxP1, proxP2, proxP3);
        auto  triangle2 = TriangleElement::create(proxP2, proxP3, proxP0);
        auto  triangle3 = TriangleElement::create(proxP3, proxP0, proxP1);

//        std::vector<BaseElement::SPtr> intersections;

//        BaseElement::SPtr inter0 = intersect_edge_triangle(e1,triangle0);
//        if (inter0 != nullptr) intersections.push_back(inter0);

//        BaseElement::SPtr inter1 = intersect_edge_triangle(e1,triangle1);
//        if (inter1 != nullptr) intersections.push_back(inter1);

//        BaseElement::SPtr inter2 = intersect_edge_triangle(e1,triangle2);
//        if (inter2 != nullptr) intersections.push_back(inter2);

//        BaseElement::SPtr inter3 = intersect_edge_triangle(e1,triangle3);
//        if (inter3 != nullptr) intersections.push_back(inter3);


//        if (intersections.size() == 1) return intersections[0];

////        // /!\ Edge elements require proximities to be created

//        else if (intersections.size() == 2) {
////            auto point1 = intersections[0]->element_cast<PointElement>();
////            auto point2 = intersections[1]->element_cast<PointElement>();
////            EdgeElement::SPtr edgeIntersect = BaseElement::create<EdgeElement>(point1->getP0(), point2->getP0());
////            return edgeIntersect;
//            return intersections[0];  // To be changed !! just for a test without the cast
//        }



//        ///////////////////////// Other method /////////////////////////
        int count = 0;
        std::pair<BaseElement::SPtr,BaseElement::SPtr> point1;
        std::pair<BaseElement::SPtr,BaseElement::SPtr> point2;

        std::pair<BaseElement::SPtr,BaseElement::SPtr> inter0 = intersect_edge_triangle(e1,triangle0);
        if ((inter0.first != nullptr) && (inter0.first != nullptr)) {
            count ++;
            point1 = inter0;
        }
#ifdef PAIR_ELMTS
        std::pair<BaseElement::SPtr,BaseElement::SPtr> inter1 = intersect_edge_triangle(e1,triangle1);
        if ((inter1.first != nullptr) && (inter1.first != nullptr)) {
            count ++;
            if (point1 == nullptr) point1 = inter1;
            else point2 = inter1;
        }

        BaseElement::SPtr inter2 = intersect_edge_triangle(e1,triangle2);
        if (inter2 != nullptr) {
            count ++;
            if (point1 == nullptr) point1 = inter2;
            else point2 = inter2;
        }

        BaseElement::SPtr inter3 = intersect_edge_triangle(e1,triangle3);
        if (inter3 != nullptr) {
            count ++;
            if (point1 == nullptr) point1 = inter3;
            else point2 = inter3;
        }


        if (count == 1) {
            auto p1 = point1->element_cast<PointElement>();
            BaseProximity::SPtr prox;
            double fact0[4];
            double fact1[4];

            if (!toolbox::TetrahedronToolBox::isInTetra(eP0, tetraInfo, fact0[0], fact0[1], fact0[2], fact0[3])) {
                if (!toolbox::TetrahedronToolBox::isInTetra(eP1, tetraInfo, fact1[0], fact1[1], fact1[2], fact1[3])) {
//                    std::cout << "point returned" << std::endl;
                    return point1;
                }
                else prox = edge->getP1();
            }
            else prox = edge->getP0();

            EdgeElement::SPtr edgeIntersect1 = BaseElement::create<EdgeElement>(p1->getP0(), prox);
//            std::cout << "edge with count == 1" << std::endl;
            return edgeIntersect1;
        }


        else if (count == 2) {
            auto p1 = point1->element_cast<PointElement>();
            auto p2 = point2->element_cast<PointElement>();
            EdgeElement::SPtr edgeIntersect2 = BaseElement::create<EdgeElement>(p1->getP0(), p2->getP0());
//            std::cout << "edge with count == 2" << std::endl;
            return edgeIntersect2;
        }


        delete triangle0;
        delete triangle1;
        delete triangle2;
        delete triangle3;
        return nullptr;
#endif
    }

    static std::pair<BaseElement::SPtr,BaseElement::SPtr> intersect_tetra_edge(BaseElement::SPtr e1, BaseElement::SPtr e2) {
        intersect_edge_tetra(e2,e1);
    }

  // /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


    static std::pair<BaseElement::SPtr,BaseElement::SPtr> intersect_edge_triangle(BaseElement::SPtr e1, BaseElement::SPtr e2) { /*std::cout << "intersect_edge_triangle" << std::endl;*/
#ifdef PAIR_ELMTS
        auto edge = e1->element_cast<EdgeElement>();
        auto triangle = e2->element_cast<TriangleElement>();


        type::Vector3 p0 = triangle->getP0()->getPosition();
        type::Vector3 edgeTri1 = triangle->getP1()->getPosition() - triangle->getP0()->getPosition();
        type::Vector3 edgeTri2 = triangle->getP2()->getPosition() - triangle->getP0()->getPosition();
        type::Vector3 d = edge->getP1()->getPosition() - edge->getP0()->getPosition();

        double cx = edge->getP0()->getPosition()[0];
        double cy = edge->getP0()->getPosition()[1];
        double cz = edge->getP0()->getPosition()[2];

        type::Vector3 h = cross(d,edgeTri2);
        double a = dot(edgeTri1,h);

        //Parallel triangle
        if (a == 0.0)  {

//        ////// for the moment ///////
//            std::cout << "edge parallel to triangle" << std::endl;
            return nullptr;

//            type::Vector3 p1 = edgeTri1+p0;
//            type::Vector3 p2 = edgeTri2+p0;
            type::Vector3 p1 = triangle->getP1()->getPosition();
            type::Vector3 p2 = triangle->getP2()->getPosition();

            double alpha[3];

//            alpha[0] = edgeIntersection(p0,p1,d,cx,cy,cz);
//            alpha[1] = edgeIntersection(p1,p2,d,cx,cy,cz);
//            alpha[2] = edgeIntersection(p2,p0,d,cx,cy,cz);

//            createLayersOnParallelTriangle(obj,tid,tri0,tri1,tri2,p0,p1,p2,cx,cy,alpha);
        }
        else {
            double f = 1.0/a;
            bool dir = a>0;

//            type::Vector3 s(cx*d_pixelSize.getValue()-p0[0],cy*d_pixelSize.getValue()-p0[1],cz*d_pixelSize.getValue()-p0[2]);
            type::Vector3 s(cx-p0[0],cy-p0[1],cz-p0[2]);

            double fact_u = f * dot(s,h);
            if (fact_u<0 || fact_u>1) return nullptr;

            type::Vector3 q = cross(s,edgeTri1);
            double fact_v = f * dot(d,q);
            if (fact_v<0 || fact_u + fact_v>1) return nullptr;

//            double depth = cz*d_pixelSize.getValue() + f*dot(edgeTri2,q);
            double depth = f*dot(edgeTri2,q);
            if (depth<0 || depth>1) return nullptr;


            TriangleProximity::SPtr triangleProx = BaseProximity::create<TriangleProximity>(triangle->getP0(),triangle->getP1(),triangle->getP2(),1-fact_u-fact_v,fact_u,fact_v);
            EdgeProximity::SPtr edgeProx = BaseProximity::create<EdgeProximity>(edge->getP0(), edge->getP1(), 1-depth, depth);
            PointElement::SPtr pointFromTriangle = BaseElement::create<PointElement>(triangleProx);
            PointElement::SPtr pointFromEdge = BaseElement::create<PointElement>(edgeProx);

            return pointFromTriangle;
        }


//        // Compute the Plucker coefficient for the edges (--> in edgeToolbox ?)
//        double pluckerT0[6];
//        double pluckerT1[6];
//        double pluckerT2[6];
//        double pluckerEdge[6];

//        compute_plucker_coeffs(pluckerT0, triangle->getP0(), triangle->getP1());
//        compute_plucker_coeffs(pluckerT1, triangle->getP1(), triangle->getP2());
//        compute_plucker_coeffs(pluckerT2, triangle->getP2(), triangle->getP0());
//        compute_plucker_coeffs(pluckerEdge, edge->getP0(), edge->getP1());


//        // Compute the side product related to the three edges of the triangle
//        double s0 = side_product(pluckerEdge,pluckerT0);
//        double s1 = side_product(pluckerEdge,pluckerT1);
//        double s2 = side_product(pluckerEdge,pluckerT2);

//        // Algorithm to compute the intersection --> calls the algo for 2D interection


        // ////////////// Other approach ////////////// // Based on https://en.wikipedia.org/wiki/Line%E2%80%93plane_intersection
//        sofa::type::Vector3 line =  edge->getP1()->getPosition() - edge->getP0()->getPosition();
//        sofa::type::Vector3 T01 = triangle->getP1()->getPosition() - triangle->getP0()->getPosition();
//        sofa::type::Vector3 T02 = triangle->getP2()->getPosition() - triangle->getP0()->getPosition();
//        double determinant = -dot(line, T01.cross(T02));
        // ////////////// ////////////// //////////////



//        return point->createProximity();
#endif
    }

    static std::pair<BaseElement::SPtr,BaseElement::SPtr> intersect_triangle_edge(BaseElement::SPtr e1, BaseElement::SPtr e2) {
        intersect_edge_triangle(e2,e1);
    }






    static std::pair<BaseElement::SPtr,BaseElement::SPtr> intersect_edge_edge(BaseElement::SPtr e1, BaseElement::SPtr e2) {
//        auto edge1 = e1->element_cast<EdgeElement>();
//        auto edge2 = e2->element_cast<EdgeElement>();

//        type::Vector3 v0 = edge1->getP0()->getPosition();
//        type::Vector3 v1 = edge1->getP1()->getPosition();
//        type::Vector3 P = v0 - v1;
//        type::Vector3 O = edge2->getP0()->getPosition();
//        type::Vector3 D = edge2->getP1()->getPosition() - edge2->getP0()->getPosition();
//        type::Vector3 T = O - v1 + type::Vector3(1,1,1);

//        sofa::type::Mat3x3d A(sofa::type::Mat3x3d::Line(-D[0], P[0], 1),
//                              sofa::type::Mat3x3d::Line(-D[1], P[1], 1),
//                              sofa::type::Mat3x3d::Line(-D[2], P[2], 1));

//        auto det = sofa::type::determinant(A);

//        if (det == 0) /*return nullptr*/; //   /!\ Depends on the rank of A and (A|b) (where b is such that Ax = b)





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

