#include <sofa/collisionAlgorithm/geometry/TriangleGeometry.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa::collisionAlgorithm {

SOFA_DECL_CLASS(TriangleGeometry)

int TriangleGeometryClass = core::RegisterObject("TriangleGeometry")
.add< TriangleGeometry<sofa::defaulttype::Vec3dTypes> >();

}
