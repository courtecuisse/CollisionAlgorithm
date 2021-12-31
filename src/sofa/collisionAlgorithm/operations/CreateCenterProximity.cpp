#include <sofa/collisionAlgorithm/operations/CreateCenterProximity.h>
#include <sofa/collisionAlgorithm/toolbox/PointToolBox.h>
#include <sofa/collisionAlgorithm/toolbox/EdgeToolBox.h>
#include <sofa/collisionAlgorithm/toolbox/TriangleToolBox.h>

namespace sofa::collisionAlgorithm::Operations {

int register_CenterProximity_Point = CreateCenterProximity::register_func<PointElement>(&toolbox::PointToolBox::createCenterProximity);

int register_CenterProximity_Edge = CreateCenterProximity::register_func<EdgeElement>(&toolbox::EdgeToolBox::createCenterProximity);

int register_CenterProximity_Triangle = CreateCenterProximity::register_func<TriangleElement>(&toolbox::TriangleToolBox::createCenterProximity);

}

