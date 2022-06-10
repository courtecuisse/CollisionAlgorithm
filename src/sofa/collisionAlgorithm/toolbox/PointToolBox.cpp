#pragma once

#include <sofa/collisionAlgorithm/toolbox/PointToolBox.h>
#include <sofa/collisionAlgorithm/proximity/PointProximity.h>

namespace sofa::collisionAlgorithm::toolbox {

Operations::CreateCenterProximity::Result PointToolBox::createCenterProximity(const PointElement::SPtr & point) {
    return point->getP0();
}

Operations::Project::Result PointToolBox::project(const type::Vector3 & P, const PointElement::SPtr & point) {
    double dist = (P-point->getP0()->getPosition()).norm();
    BaseProximity::SPtr prox = point->getP0();

    return Operations::Project::Result(dist,prox);
}

}

