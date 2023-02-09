#include <sofa/collisionAlgorithm/broadphase/FullAABBBroadPhase.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa::collisionAlgorithm {

SOFA_DECL_CLASS(FullAABBBroadPhase)

int FullAABBBroadPhaseClass = core::RegisterObject("FullAABBBroadPhase")
.add< FullAABBBroadPhase >();

}
