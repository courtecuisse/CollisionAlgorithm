#pragma once

#include <sofa/collisionAlgorithm/BaseProximity.h>
#include <sofa/collisionAlgorithm/elements/EdgeElement.h>
#include <sofa/collisionAlgorithm/proximity/EdgeProximity.h>

namespace sofa::collisionAlgorithm::toolbox {

class EdgeToolBox {
public:

    static typename BaseProximity::SPtr createCenterProximity(BaseElement::SPtr elmt) {
        auto edge = std::static_pointer_cast<EdgeElement>(elmt);
        return EdgeProximity::create(edge,0.5,0.5);
    }

    static BaseProximity::SPtr project(const type::Vector3 & P, BaseElement::SPtr elmt) {
        auto edge = std::static_pointer_cast<EdgeElement>(elmt);

        double fact_u,fact_v;
        const type::Vector3 P0 = EdgeProximity::create(edge,1,0)->getPosition();
        const type::Vector3 P1 = EdgeProximity::create(edge,0,1)->getPosition();

        projectOnEdge(P,P0,P1,fact_u,fact_v);

        return EdgeProximity::create(edge,fact_u,fact_v);
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

