#include <sofa/collisionAlgorithm/operations/Project.h>
#include <sofa/collisionAlgorithm/toolbox/PointToolBox.h>
#include <sofa/collisionAlgorithm/toolbox/EdgeToolBox.h>
#include <sofa/collisionAlgorithm/toolbox/TriangleToolBox.h>
#include <sofa/collisionAlgorithm/toolbox/TetrahedronToolBox.h>

namespace sofa::collisionAlgorithm::Operations {

int register_Project_Point = ProjectOperation::register_func<PointElement>(&toolbox::PointToolBox::project);

int register_Project_Edge = ProjectOperation::register_func<EdgeElement>(&toolbox::EdgeToolBox::project);

int register_Project_Triangle = ProjectOperation::register_func<TriangleElement>(&toolbox::TriangleToolBox::project);

int register_Project_Tetrahedron = ProjectOperation::register_func<TetrahedronElement>(&toolbox::TetrahedronToolBox::project);

}

