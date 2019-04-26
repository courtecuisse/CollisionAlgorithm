#include <sofa/collisionAlgorithm/data/ContactDistance.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa {

namespace collisionAlgorithm {

SOFA_DECL_CLASS(ContactDistance);

int ContactDistanceClass = core::RegisterObject("ContactDistance")
.add< ContactDistance >();

}

}

