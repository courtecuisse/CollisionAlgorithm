#pragma once

//#include <sofa/collisionAlgorithm/BaseProximity.h>
#include <sofa/collisionAlgorithm/elements/PointElement.h>
#include <sofa/collisionAlgorithm/elements/EdgeElement.h>
#include <sofa/collisionAlgorithm/elements/TriangleElement.h>
#include <sofa/collisionAlgorithm/elements/TetrahedronElement.h>
#include <sofa/collisionAlgorithm/BaseElement.h>
#include <sofa/collisionAlgorithm/toolbox/EdgeToolBox.h>
// #include <sofa/component/topology/container/dynamic/EdgeSetTopologyModifier.h>
//#include <EdgeSetTopologyModifier.h>
//#include <EdgeSetGeometryAlgorithm.h>

namespace sofa::collisionAlgorithm::toolbox {

class IntersectionToolBox {
public:


    static BaseElement::SPtr intersect_edge_tetra(BaseElement* e1, BaseElement* e2) {
//        auto edge = e1->element_cast<EdgeElement>();
//        auto tetra = e2->element_cast<TetraElement>();
//        return point->createProximity();
    }

    static BaseElement::SPtr intersect_tetra_edge(BaseElement* e1, BaseElement* e2) {
        intersect_edge_tetra(e2,e1);
    }

  // /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    static BaseElement::SPtr intersect_edge_triangle(BaseElement* e1, BaseElement* e2) {
//        auto edge = e1->element_cast<EdgeElement>();
//        auto triangle = e2->element_cast<TriangleElement>();


        // for each edge of the triangle, compute intersect_edge_edge(edge, edge_tri)
        // If 2 intersections :


//        return point->createProximity();
    }

    static BaseElement::SPtr intersect_triangle_edge(BaseElement* e1, BaseElement* e2) {
        intersect_edge_triangle(e2,e1);
    }






    static BaseElement::SPtr intersect_edge_edge(BaseElement* e1, BaseElement* e2) {
//        auto edge1 = e1->element_cast<EdgeElement>();
//        auto edge2 = e2->element_cast<EdgeElement>();

//        auto a1 = edge1->getP0();
//        auto b1 = edge1->getP1();
//        auto a2 = edge2->getP0();
//        auto b2 = edge2->getP1();


//        type::Vec3d B = a2 - a1;
//        sofa::type::Mat<3,2,double> A;

        // Compute intersection coordinates

        // find barycentric coord of that point in the edge

        // Create proximity from those bary coord



//        return ///;
    }

};



}

