#include <sofa/collisionAlgorithm/geometry/BezierTriangleGeometry.h>
#include <sofa/collisionAlgorithm/normals/BezierTriangleNormalHandler.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa {

namespace collisionAlgorithm {

SOFA_DECL_CLASS(BezierTriangleNormalHandler)

int BezierTriangleNormalHandlerClass = core::RegisterObject("BezierTriangleNormalHandler")
.add< BezierTriangleNormalHandler<BezierTriangleGeometry<sofa::defaulttype::Vec3dTypes> > >();

}

}
