#include <sofa/collisionAlgorithm/distanceMeasures/ProjectiveDistanceMeasure.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa {

namespace collisionAlgorithm {

SOFA_DECL_CLASS (ProjectiveDistanceMeasure) ;

int ProjectiveDistanceMeasureClass = core::RegisterObject("ProjectiveDistanceMeasure")
.add< ProjectiveDistanceMeasure >();

}

}
