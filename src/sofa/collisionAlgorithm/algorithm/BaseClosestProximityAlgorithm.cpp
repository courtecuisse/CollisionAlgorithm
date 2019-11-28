#include <sofa/collisionAlgorithm/algorithm/BaseClosestProximityAlgorithm.inl>
#include <sofa/core/ObjectFactory.h>

namespace sofa {

namespace collisionAlgorithm {

SOFA_DECL_CLASS(Distance3DProximityMeasure)

int Distance3DProximityMeasureClass = core::RegisterObject("Default implementation of a Distance in 3D between 2 Proximities")
.add< Distance3DProximityMeasure >();



}

}



