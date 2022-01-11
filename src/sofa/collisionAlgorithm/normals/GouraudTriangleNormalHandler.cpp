#include <sofa/collisionAlgorithm/normals/GouraudTriangleNormalHandler.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa::collisionAlgorithm {

SOFA_DECL_CLASS(GouraudTriangleNormalHandler)

int GouraudTriangleNormalHandlerClass = core::RegisterObject("GouraudTriangleNormalHandler")
.add< GouraudTriangleNormalHandler<sofa::defaulttype::Vec3dTypes> >();

}
