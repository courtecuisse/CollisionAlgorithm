#include <sofa/collisionAlgorithm/normals/DefaultEdgeNormalHandler.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa {

namespace collisionAlgorithm {

SOFA_DECL_CLASS(DefaultEdgeNormalHandler)

int DefaultEdgeNormalHandlerClass = core::RegisterObject("Default implementation of normal computations return normal in the direction of the edge")
.add< DefaultEdgeNormalHandler<sofa::defaulttype::Vec3dTypes> >();

}

}
