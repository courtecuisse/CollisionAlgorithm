#include <sofa/collisionAlgorithm/geometry/EdgeGeometry.inl>
#include <sofa/core/ObjectFactory.h>

namespace sofa {

namespace collisionAlgorithm {

SOFA_DECL_CLASS(EdgeGeometry)

int EdgeGeometryClass = core::RegisterObject("EdgeGeometry")
.add< EdgeGeometry<sofa::defaulttype::Vec3dTypes> >();

}

}
