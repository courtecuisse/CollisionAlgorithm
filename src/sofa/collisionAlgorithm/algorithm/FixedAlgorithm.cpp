#include <sofa/collisionAlgorithm/algorithm/FixedAlgorithm.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa {

namespace collisionAlgorithm {

SOFA_DECL_CLASS(FixedAlgorithm)

int FixedAlgorithmClass = core::RegisterObject("FixedAlgorithm")
.add< FixedAlgorithm >();

}

}



