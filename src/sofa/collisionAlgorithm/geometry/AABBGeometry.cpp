#include <sofa/collisionAlgorithm/geometry/AABBGeometry.inl>
#include <sofa/core/ObjectFactory.h>

namespace sofa {

namespace collisionAlgorithm {

SOFA_DECL_CLASS(AABBGeometry)

int AABBGeometryClass = core::RegisterObject("AABBGeometry")
.add< AABBGeometry >();

}

}
