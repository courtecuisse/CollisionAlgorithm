#include <sofa/collisionAlgorithm/geometry/PointGeometry.h>
#include <sofa/collisionAlgorithm/normals/GravityPointNormalHandler.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa::collisionAlgorithm {

SOFA_DECL_CLASS(GravityPointNormalHandler)

int GravityPointNormalHandlerClass = core::RegisterObject("return the normal between the gravity center of the object and each point")
.add< GravityPointNormalHandler<sofa::defaulttype::Vec3dTypes> >();

}
