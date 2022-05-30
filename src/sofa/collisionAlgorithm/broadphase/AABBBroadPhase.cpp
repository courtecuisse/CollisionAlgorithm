#include <sofa/collisionAlgorithm/broadphase/AABBBroadPhase.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa::collisionAlgorithm {

SOFA_DECL_CLASS(AABBBroadPhase)

int AABBBroadPhaseClass = core::RegisterObject("AABBBroadPhase")
.add< AABBBroadPhase >();

}
