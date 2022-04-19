#include <sofa/collisionAlgorithm/algorithm/ElementsIntersectionAlgorithm.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa::collisionAlgorithm {

SOFA_DECL_CLASS(FindClosestProximityAlgorithm)

int ElementsIntersectionAlgorithmClass = core::RegisterObject("ElementsIntersectionAlgorithm")
.add< ElementsIntersectionAlgorithm >();

}



