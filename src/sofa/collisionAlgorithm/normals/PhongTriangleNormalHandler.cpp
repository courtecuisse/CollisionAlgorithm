#include <sofa/collisionAlgorithm/normals/PhongTriangleNormalHandler.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa::collisionAlgorithm {

SOFA_DECL_CLASS(PhongTriangleNormalHandler)

int PhongTriangleNormalHandlerClass = core::RegisterObject("PhongTriangleNormalHandler")
.add< PhongTriangleNormalHandler<sofa::defaulttype::Vec3dTypes> >();

}
