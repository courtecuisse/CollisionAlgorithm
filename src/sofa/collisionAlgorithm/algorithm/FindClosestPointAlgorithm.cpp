#include <sofa/collisionAlgorithm/algorithm/FindClosestPointAlgorithm.inl>
#include <sofa/core/ObjectFactory.h>

namespace sofa {

namespace collisionAlgorithm {

SOFA_DECL_CLASS(FindClosestPointAlgorithm)

int FindClosestPointAlgorithmClass = core::RegisterObject("FindClosestPointAlgorithm")
.add< FindClosestPointAlgorithm >();

}

}



