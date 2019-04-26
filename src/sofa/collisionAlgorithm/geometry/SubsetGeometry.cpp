#include <sofa/collisionAlgorithm/geometry/SubsetGeometry.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa {

namespace collisionAlgorithm {

SOFA_DECL_CLASS(SubsetGeometry)

int SubsetGeometryClass = core::RegisterObject("SubsetGeometry")
.add< SubsetGeometry<sofa::defaulttype::Vec3dTypes> >();

}

}
