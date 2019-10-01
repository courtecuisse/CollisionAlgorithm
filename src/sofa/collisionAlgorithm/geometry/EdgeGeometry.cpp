#include <sofa/collisionAlgorithm/geometry/EdgeGeometry.h>
#include <sofa/collisionAlgorithm/normals/DefaultEdgeNormalHandler.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa {

namespace collisionAlgorithm {

template<class DataTypes>
void EdgeGeometry<DataTypes>::bwdInit() {
    if (m_normalHandler != NULL) return;

    auto handler = sofa::core::objectmodel::New<DefaultEdgeNormalHandler<DataTypes> >();
    handler->setName("normals");
    handler->l_geometry.set(this);
    handler->init();
}

SOFA_DECL_CLASS(EdgeGeometry)

int EdgeGeometryClass = core::RegisterObject("EdgeGeometry")
.add< EdgeGeometry<sofa::defaulttype::Vec3dTypes> >();

}

}
