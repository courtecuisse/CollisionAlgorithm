#include <sofa/collisionAlgorithm/normals/TrajectoryNormalHandler.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa::collisionAlgorithm {

SOFA_DECL_CLASS(EdgeNormalHandler)

int TrajectoryNormalHandlerClass = core::RegisterObject("Default implementation of normal computations return normal in the direction of the edge")
.add< TrajectoryNormalHandler<sofa::defaulttype::Vec3dTypes > >();

}
