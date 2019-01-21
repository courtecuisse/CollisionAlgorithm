#include <sofa/collisionAlgorithm/geometry/PhongTriangleGeometry.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa {

namespace collisionAlgorithm {

SOFA_DECL_CLASS(PhongTriangleGeometry)

int PhongTriangleGeometryClass = core::RegisterObject("PhongTriangleGeometry")
.add< PhongTriangleGeometry<sofa::defaulttype::Vec3dTypes> >();

}

}
