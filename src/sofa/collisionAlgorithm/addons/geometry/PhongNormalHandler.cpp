#include <sofa/collisionAlgorithm/addons/geometry/PhongNormalHandler.h>
#include <sofa/collisionAlgorithm/BaseGeometryModifier.h>
#include <sofa/collisionAlgorithm/geometry/TriangleGeometry.h>
#include <sofa/collisionAlgorithm/element/TriangleElement.h>

#include <sofa/core/ObjectFactory.h>

namespace sofa
{

namespace collisionAlgorithm
{

template<class TGeometry>
defaulttype::Vector3 PhongNormalHandler<TGeometry>::internalGetNormal(const BaseGeometry*, const size_t, const double* ) const
{
    msg_error("PhongNormalHandler") << "Geometry does not implement Phong Normal";
    return defaulttype::Vector3();
}

template<>
defaulttype::Vector3 PhongNormalHandler<TriangleGeometry>::internalGetNormal(const BaseGeometry* geometry, const size_t id, const double* fact) const
{
    const std::vector<defaulttype::Vector3> & normals = static_cast<const TriangleGeometry*>(geometry)->pointNormals();
    const TriangleElement* element = static_cast<const TriangleElement*>(geometry->getElement(id));

    return normals[element->pointIDs()[0]] * fact[0] +
           normals[element->pointIDs()[1]] * fact[1] +
           normals[element->pointIDs()[2]] * fact[2];
}


int PhongNormalHandlerClass = core::RegisterObject("PhongNormalHandler")
.add< SofaPhongNormalHandler >()
.addAlias("PhongNormalHandler");

SofaPhongNormalHandler::SofaPhongNormalHandler()
: SofaBaseNormalHandler()
{
    if(dynamic_cast<TriangleGeometry*>(l_geometry.get()))
    {
        m_impl = new PhongNormalHandler<TriangleGeometry>();
    }

}

SofaPhongNormalHandler::~SofaPhongNormalHandler()
{

}

}

}
