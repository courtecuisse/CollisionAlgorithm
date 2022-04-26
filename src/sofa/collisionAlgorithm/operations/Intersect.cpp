#include <sofa/collisionAlgorithm/operations/Intersect.h>
#include <sofa/collisionAlgorithm/toolbox/PointToolBox.h>
#include <sofa/collisionAlgorithm/toolbox/EdgeToolBox.h>
#include <sofa/collisionAlgorithm/toolbox/TriangleToolBox.h>
#include <sofa/collisionAlgorithm/toolbox/TetrahedronToolBox.h>
#include <sofa/collisionAlgorithm/toolbox/IntersectionToolBox.h>

namespace sofa::collisionAlgorithm::Operations {

int register_Intersect_Edge_Tetra = IntersectOperation::register_func<EdgeElement,TetrahedronElement>(&toolbox::IntersectionToolBox::intersect_edge_tetra);
int register_Intersect_Tetra_Edge = IntersectOperation::register_func<TetrahedronElement,EdgeElement>(&toolbox::IntersectionToolBox::intersect_tetra_edge);

int register_Intersect_Edge_Triangle = IntersectOperation::register_func<EdgeElement,TriangleElement>(&toolbox::IntersectionToolBox::intersect_edge_triangle);
int register_Intersect_Triangle_Edge = IntersectOperation::register_func<TriangleElement,EdgeElement>(&toolbox::IntersectionToolBox::intersect_triangle_edge);

int register_Intersect_Edge_Edge = IntersectOperation::register_func<EdgeElement,EdgeElement>(&toolbox::IntersectionToolBox::intersect_edge_edge);

}

