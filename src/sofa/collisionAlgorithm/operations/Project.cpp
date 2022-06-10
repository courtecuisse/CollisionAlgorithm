#include <sofa/collisionAlgorithm/operations/Project.h>
#include <sofa/collisionAlgorithm/toolbox/PointToolBox.h>
#include <sofa/collisionAlgorithm/toolbox/EdgeToolBox.h>
#include <sofa/collisionAlgorithm/toolbox/TriangleToolBox.h>
#include <sofa/collisionAlgorithm/toolbox/TetrahedronToolBox.h>

namespace sofa::collisionAlgorithm::Operations::Project {

int register_Project_Point = Operation::register_func<PointElement>(&toolbox::PointToolBox::project);

int register_Project_Edge = Operation::register_func<EdgeElement>(&toolbox::EdgeToolBox::project);

int register_Project_Triangle = Operation::register_func<TriangleElement>(&toolbox::TriangleToolBox::project);

int register_Project_Tetrahedron = Operation::register_func<TetrahedronElement>(&toolbox::TetrahedronToolBox::project);

}

