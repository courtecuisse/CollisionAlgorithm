#include <sofa/collisionAlgorithm/geometry/PointGeometry.h>
#include <sofa/collisionAlgorithm/normals/DefaultPointNormalHandler.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa {

namespace collisionAlgorithm {

template<class DataTypes>
void PointGeometry<DataTypes>::bwdInit() {
    if (m_normalHandler != NULL) return;

    auto handler = sofa::core::objectmodel::New<DefaultPointNormalHandler<DataTypes> >();
    handler->setName("normals");
    handler->l_geometry.set(this);
    handler->init();
}

SOFA_DECL_CLASS(PointGeometry)

int PointGeometryClass = core::RegisterObject("PointGeometry")
.add< PointGeometry<sofa::defaulttype::Vec3dTypes> >();

}

}


