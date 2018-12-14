#include <sofa/collisionAlgorithm/geometry/BezierTriangleGeometry.inl>
#include <sofa/core/ObjectFactory.h>

namespace sofa
{

namespace collisionAlgorithm
{

SOFA_DECL_CLASS(BezierTriangleGeometry)

int BezierTriangleGeometryClass = core::RegisterObject("BezierTriangleGeometry")
.add< BezierTriangleGeometry >();

}

}
