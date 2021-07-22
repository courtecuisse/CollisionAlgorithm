#include <sofa/collisionAlgorithm/normals/VectorPointNormalHandler.h>
#include <sofa/collisionAlgorithm/proximity/PointProximity.h>
#include <sofa/collisionAlgorithm/proximity/EdgeProximity.h>
#include <sofa/collisionAlgorithm/proximity/TriangleProximity.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa {

namespace collisionAlgorithm {

SOFA_DECL_CLASS(VectorPointNormalHandler)

int VectorPointNormalHandlerClass = core::RegisterObject("Default implementation of normal computation for point geometry")
.add< VectorPointNormalHandler<TBaseGeometry<sofa::defaulttype::Vec3dTypes,TriangleProximity> > >(true)
.add< VectorPointNormalHandler<TBaseGeometry<sofa::defaulttype::Vec3dTypes,PointProximity> > >()
.add< VectorPointNormalHandler<TBaseGeometry<sofa::defaulttype::Vec3dTypes,EdgeProximity> > >();

}

}
