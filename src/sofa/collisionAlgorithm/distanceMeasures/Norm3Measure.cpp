#include <sofa/collisionAlgorithm/distanceMeasures/Norm3Measure.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa {

namespace collisionAlgorithm {

SOFA_DECL_CLASS (Norm3Measure) ;

int Norm3MeasureClass = core::RegisterObject("Norm3Measure")
.add< Norm3Measure >();

}

}
