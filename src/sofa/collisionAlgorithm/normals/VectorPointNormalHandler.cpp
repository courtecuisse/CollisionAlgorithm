#include <sofa/collisionAlgorithm/normals/VectorPointNormalHandler.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa::collisionAlgorithm {

SOFA_DECL_CLASS(VectorPointNormalHandler)

int VectorPointNormalHandlerClass = core::RegisterObject("Default implementation of normal computation for point geometry")
.add< VectorPointNormalHandler<sofa::defaulttype::Vec3dTypes> >();

}
