#include <sofa/collisionAlgorithm/utils/ProximityPairHolder.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa {

namespace collisionAlgorithm {

SOFA_DECL_CLASS(ProximityPairHolder)

int ProximityPairHolderClass = core::RegisterObject("ProximityPairHolder")
.add< ProximityPairHolder >();

}

}



