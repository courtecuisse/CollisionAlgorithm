#include <sofa/collisionAlgorithm/geometry/AABBBroadPhase.inl>
#include <sofa/core/ObjectFactory.h>

namespace sofa {

namespace collisionAlgorithm {

SOFA_DECL_CLASS(AABBBroadPhase)

int AABBBroadPhaseClass = core::RegisterObject("AABBBroadPhase")
.add< AABBBroadPhase >();

}

}
