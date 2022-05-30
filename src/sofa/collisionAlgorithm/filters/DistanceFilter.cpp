#include <sofa/collisionAlgorithm/filters/DistanceFilter.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa::collisionAlgorithm {

int DistanceFilterClass = core::RegisterObject("DistanceFilter")
.add< DistanceFilter >();

}
