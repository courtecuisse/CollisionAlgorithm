#include <sofa/collisionAlgorithm/addons/geometry/PhongNormalHandler.h>
#include <sofa/collisionAlgorithm/BaseGeometryModifier.h>
#include <sofa/collisionAlgorithm/geometry/TriangleGeometry.h>

#include <sofa/core/ObjectFactory.h>

namespace sofa
{

namespace collisionAlgorithm
{

//template<>
//defaulttype::Vector3 SofaPhongNormalHandler<TriangleGeometry>::getNormal(const size_t id, const double* fact) const
//{
//    const std::vector<defaulttype::Vector3> & normals = static_cast<const TriangleGeometry*>(m_geometry)->pointNormals();
//    const TriangleElement* element = static_cast<const TriangleElement*>(m_geometry->getElement(id));

//    return normals[element->pointIDs()[0]] * fact[0] +
//           normals[element->pointIDs()[1]] * fact[1] +
//           normals[element->pointIDs()[2]] * fact[2];
//}


//int PhongNormalHandlerClass = core::RegisterObject("PhongNormalHandler")
//.add< SofaPhongNormalHandler<TriangleGeometry> >()
//.addAlias("PhongNormalHandler");


}

}
