#include <geometry/PointGeometry.inl>
#include <sofa/core/ObjectFactory.h>

namespace sofa {

namespace collisionAlgorithm {

SOFA_DECL_CLASS(PointGeometry)

int PointGeometryClass = core::RegisterObject("PointGeometry")
.add< PointGeometry >();

}

}


