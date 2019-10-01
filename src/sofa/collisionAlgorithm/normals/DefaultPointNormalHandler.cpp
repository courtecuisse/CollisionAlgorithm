#include <sofa/collisionAlgorithm/normals/DefaultPointNormalHandler.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa {

namespace collisionAlgorithm {

SOFA_DECL_CLASS(DefaultPointNormalHandler)

int DefaultPointNormalHandlerClass = core::RegisterObject("Default implementation of normal computations return null normal for point geometries")
.add< DefaultPointNormalHandler<sofa::defaulttype::Vec3dTypes> >();

}

}
