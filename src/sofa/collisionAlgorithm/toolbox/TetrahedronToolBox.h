#pragma once

#include <sofa/collisionAlgorithm/BaseProximity.h>
#include <sofa/collisionAlgorithm/toolbox/TriangleToolBox.h>
#include <sofa/collisionAlgorithm/elements/TetrahedronElement.h>

namespace sofa::collisionAlgorithm::toolbox {

class TetrahedronToolBox {
public:

    static BaseProximity::SPtr createCenterProximity(BaseElement::SPtr elmt) {
        TetrahedronElement * tetra = elmt->cast<TetrahedronElement>();
        return tetra->createProximity(1.0/4.0,1.0/4.0,1.0/4.0,1.0/4.0);
    }

    //Barycentric coordinates are computed according to
    //http://gamedev.stackexchange.com/questions/23743/whats-the-most-efficient-way-to-find-barycentric-coordinates
    static BaseProximity::SPtr project(type::Vector3 P, BaseElement::SPtr elmt) {

    }

};



}

