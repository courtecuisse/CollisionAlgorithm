#include <sofa/collisionAlgorithm/geometry/TriangleGeometry.inl>
#include <sofa/core/ObjectFactory.h>

namespace sofa {

namespace collisionAlgorithm {

SOFA_DECL_CLASS(TriangleGeometry)

int TriangleGeometryClass = core::RegisterObject("TriangleGeometry")
.add< TriangleGeometry >();

}

}
