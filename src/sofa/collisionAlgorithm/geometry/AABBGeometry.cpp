#include <sofa/collisionAlgorithm/geometry/AABBGeometry.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa::collisionAlgorithm {

SOFA_DECL_CLASS(AABBGeometry)

int AABBGeometryClass = core::RegisterObject("AABBGeometry")
.add< AABBGeometry >();

}
