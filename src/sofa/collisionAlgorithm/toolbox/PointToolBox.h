#pragma once

#include <sofa/collisionAlgorithm/BaseProximity.h>
#include <sofa/collisionAlgorithm/elements/PointElement.h>
#include <sofa/collisionAlgorithm/operations/Project.h>
#include <sofa/collisionAlgorithm/operations/CreateCenterProximity.h>

namespace sofa::collisionAlgorithm::toolbox {

class PointToolBox {
public:

    static Operations::CreateCenterProximity::Result createCenterProximity(const PointElement::SPtr & point);

    static Operations::Project::Result project(const type::Vec3 & P, const PointElement::SPtr & point);

};



}

