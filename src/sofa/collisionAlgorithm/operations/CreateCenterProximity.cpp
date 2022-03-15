#include <sofa/collisionAlgorithm/operations/CreateCenterProximity.h>
#include <sofa/collisionAlgorithm/toolbox/PointToolBox.h>
#include <sofa/collisionAlgorithm/toolbox/EdgeToolBox.h>
#include <sofa/collisionAlgorithm/toolbox/TriangleToolBox.h>
#include <sofa/collisionAlgorithm/toolbox/TetrahedronToolBox.h>

namespace sofa::collisionAlgorithm::Operations {

int register_CenterProximity_Point = CreateCenterProximityOperation::register_func<PointElement>(&toolbox::PointToolBox::createCenterProximity);

int register_CenterProximity_Edge = CreateCenterProximityOperation::register_func<EdgeElement>(&toolbox::EdgeToolBox::createCenterProximity);

int register_CenterProximity_Triangle = CreateCenterProximityOperation::register_func<TriangleElement>(&toolbox::TriangleToolBox::createCenterProximity);

int register_CenterProximity_Tetrahedron = CreateCenterProximityOperation::register_func<TetrahedronElement>(&toolbox::TetrahedronToolBox::createCenterProximity);

}

