#include <sofa/collisionAlgorithm/CollisionPipeline.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa::collisionAlgorithm {

SOFA_DECL_CLASS(CollisionLoop)

int CollisionLoopClass = core::RegisterObject("CollisionLoop")
.add< CollisionLoop >();

}
