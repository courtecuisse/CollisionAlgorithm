#include <sofa/collisionAlgorithm/geometry/FixedGeometry.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa {

namespace collisionAlgorithm {

SOFA_DECL_CLASS(FixedGeometry)

int FixedGeometryClass = core::RegisterObject("FixedGeometry")
.add< FixedGeometry<sofa::defaulttype::Vec3dTypes> >();

}

}


