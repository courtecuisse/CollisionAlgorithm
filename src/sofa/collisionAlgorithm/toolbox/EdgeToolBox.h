#pragma once

#include <sofa/collisionAlgorithm/BaseProximity.h>
#include <sofa/collisionAlgorithm/elements/EdgeElement.h>

namespace sofa::collisionAlgorithm::toolbox {

class EdgeToolBox {
public:

    static BaseProximity::SPtr createCenterProximity(BaseElement::SPtr elmt) {
        EdgeElement::SPtr edge = elmt->edgeElements()[0];
        return edge.createProximity(0.5,0.5);
    }

    static BaseProximity::SPtr project(const type::Vector3 & P, BaseElement::SPtr elmt) {
        EdgeElement::SPtr edge = elmt->edgeElements()[0];

        double fact_u,fact_v;
        const type::Vector3 P0 = edge.createProximity(1,0)->getPosition();
        const type::Vector3 P1 = edge.createProximity(0,1)->getPosition();

        projectOnEdge(P,P0,P1,fact_u,fact_v);

        return edge.createProximity(fact_u,fact_v);
    }

    static void projectOnEdge(const type::Vec3d & projP, const type::Vec3d & e1, const type::Vec3d & e2, double & fact_u, double & fact_v) {
        type::Vec3d v = e2 - e1;
        fact_v = dot(projP - e1,v) / dot(v,v);

        if (fact_v<0.0) fact_v = 0.0;
        else if (fact_v>1.0) fact_v = 1.0;

        fact_u = 1.0-fact_v;
    }

};



}

