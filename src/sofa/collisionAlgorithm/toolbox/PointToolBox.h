#pragma once

#include <sofa/collisionAlgorithm/BaseProximity.h>
#include <sofa/collisionAlgorithm/elements/PointElement.h>
#include <sofa/collisionAlgorithm/proximity/PointProximity.h>

namespace sofa::collisionAlgorithm::toolbox {

class PointToolBox {
public:

    static BaseProximity::SPtr createCenterProximity(PointElement::SPtr point) {
        return PointProximity::create(point);
    }


    static BaseProximity::SPtr project(const type::Vector3 & /*P*/, PointElement::SPtr point) {
        return PointProximity::create(point);
    }


//    static BaseElement::SPtr intersect_edge_tetra(BaseElement::SPtr e1, BaseElement::SPtr e2) {
// //        auto edge = e1->element_cast<EdgeElement>();
// //        auto tetra = e2->element_cast<TetraElement>();
// //        return point->createProximity();
//    }

//    static BaseElement::SPtr intersect_tetra_edge(BaseElement::SPtr e1, BaseElement::SPtr e2) {
//        intersect_edge_tetra(e2,e1);
//    }

//  // /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//    static BaseElement::SPtr intersect_edge_triangle(BaseElement::SPtr e1, BaseElement::SPtr e2) {
// //        auto edge = e1->element_cast<EdgeElement>();
// //        auto triangle = e2->element_cast<TriangleElement>();
// //        return point->createProximity();
//    }

//    static BaseElement::SPtr intersect_triangle_edge(BaseElement::SPtr e1, BaseElement::SPtr e2) {
//        intersect_edge_triangle(e2,e1);
//    }





//    static BaseElement::SPtr intersect_edge_point(BaseElement::SPtr e1, BaseElement::SPtr e2) {
// //        auto edge = e1->element_cast<EdgeElement>();
// //        auto point = e2->element_cast<PointElement>();
// //        return point->createProximity();
//    }

//    static BaseElement::SPtr intersect_point_edge(BaseElement::SPtr e1, BaseElement::SPtr e2) {
//        intersect_edge_point(e2,e1);
//    }





//    static BaseElement::SPtr intersect_edge_edge(BaseElement::SPtr e1, BaseElement::SPtr e2) {
// //        auto edge1 = e1->element_cast<EdgeElement>();
// //        auto edge2 = e2->element_cast<EdgeElement>();
// //        return point->createProximity();
//    }

};



}

