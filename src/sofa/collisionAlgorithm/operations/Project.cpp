#include <sofa/collisionAlgorithm/operations/Project.h>
#include <sofa/collisionAlgorithm/toolbox/PointToolBox.h>
#include <sofa/collisionAlgorithm/toolbox/EdgeToolBox.h>
#include <sofa/collisionAlgorithm/toolbox/TriangleToolBox.h>

namespace sofa::collisionAlgorithm::Operations {

int register_Project_Point = Project::register_func<PointElement>(&toolbox::PointToolBox::project);

int register_Project_Edge = Project::register_func<EdgeElement>(&toolbox::EdgeToolBox::project);

int register_Project_Triangle = Project::register_func<TriangleElement>(&toolbox::TriangleToolBox::project);

}

