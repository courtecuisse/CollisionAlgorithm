#include <sofa/collisionAlgorithm/geometry/BezierTriangleGeometry.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa
{

namespace collisionAlgorithm
{

SOFA_DECL_CLASS(BezierTriangleGeometry)

int BezierTriangleGeometryClass = core::RegisterObject("BezierTriangleGeometry")
.add< BezierTriangleGeometry<sofa::defaulttype::Vec3dTypes> >();

}

}
