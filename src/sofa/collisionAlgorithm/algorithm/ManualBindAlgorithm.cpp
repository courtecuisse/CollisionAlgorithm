#include <sofa/collisionAlgorithm/algorithm/ManualBindAlgorithm.h>
#include <sofa/collisionAlgorithm/geometry/AABBGeometry.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa {

namespace collisionAlgorithm {

SOFA_DECL_CLASS(ManualBindAlgorithm)

int ManualBindAlgorithmClass = core::RegisterObject("ManualBindAlgorithm")
.add< ManualBindAlgorithm >();

}

}



