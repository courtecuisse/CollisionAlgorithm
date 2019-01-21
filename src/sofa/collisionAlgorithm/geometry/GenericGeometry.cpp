#include <sofa/collisionAlgorithm/geometry/GenericGeometry.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa {

namespace collisionAlgorithm {

SOFA_DECL_CLASS(GenericGeometry)

int GenericGeometryClass = core::RegisterObject("GenericGeometry")
.add< GenericGeometry<sofa::defaulttype::Vec3dTypes> >();

}

}
