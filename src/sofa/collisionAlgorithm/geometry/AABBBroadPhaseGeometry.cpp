#include <sofa/collisionAlgorithm/geometry/AABBBroadPhaseGeometry.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa {

namespace collisionAlgorithm {

SOFA_DECL_CLASS(AABBBroadPhaseGeometry)

int AABBBroadPhaseGeometryClass = core::RegisterObject("AABBBroadPhaseGeometry")
.add< AABBBroadPhaseGeometry >();

}

}
