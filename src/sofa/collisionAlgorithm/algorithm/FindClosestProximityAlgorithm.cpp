#include <sofa/collisionAlgorithm/algorithm/FindClosestProximityAlgorithm.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa {

namespace collisionAlgorithm {

SOFA_DECL_CLASS(FindClosestProximityAlgorithm)

int FindClosestPointAlgorithmClass = core::RegisterObject("FindClosestProximityAlgorithm")
.add< FindClosestProximityAlgorithm >();

}

}



