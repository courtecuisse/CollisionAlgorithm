#pragma once

#include <sofa/collisionAlgorithm/BaseProximity.h>
#include <sofa/collisionAlgorithm/elements/EdgeElement.h>

namespace sofa::collisionAlgorithm::toolbox {

class EdgeToolBox {
public:

    static BaseProximity::SPtr createCenterProximity(BaseElement::SPtr elmt) {
        auto edge = elmt->cast<EdgeElement>();
        return edge->createProximity(0.5,0.5);
    }

    static BaseProximity::SPtr project(const BaseProximity::SPtr & P, BaseElement::SPtr elmt) {
        auto edge = elmt->cast<EdgeElement>();

        double fact_u,fact_v;
        const type::Vector3 P0 = edge->getP0();
        const type::Vector3 P1 = edge->getP1();

        projectOnEdge(P->getPosition(),P0,P1,fact_u,fact_v);

        return edge->createProximity(fact_u,fact_v);
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

