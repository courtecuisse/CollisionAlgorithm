#include <algorithm/CollisionDetectionAlgorithm.inl>
#include <sofa/core/ObjectFactory.h>

namespace sofa {

namespace collisionAlgorithm {

SOFA_DECL_CLASS(CollisionDetectionAlgorithm)

int CollisionDetectionAlgorithmClass = core::RegisterObject("CollisionDetectionAlgorithm")
.add< CollisionDetectionAlgorithm >();

}

}



