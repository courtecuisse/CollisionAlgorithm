#include <sofa/collisionAlgorithm/geometry/TriangleGeometry.h>
#include <sofa/collisionAlgorithm/normals/TriangleNormalHandler.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa {

namespace collisionAlgorithm {

SOFA_DECL_CLASS(TriangleNormalHandler)

int TriangleNormalHandlerClass = core::RegisterObject("Default implementation of normal computations return normal in the direction of the edge")
.add< TriangleNormalHandler<TriangleGeometry<sofa::defaulttype::Vec3dTypes> > >();

}

}
