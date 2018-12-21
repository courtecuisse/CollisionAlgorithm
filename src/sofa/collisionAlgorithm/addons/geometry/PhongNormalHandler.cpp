#include <sofa/collisionAlgorithm/addons/geometry/PhongNormalHandler.h>
#include <sofa/collisionAlgorithm/BaseGeometryModifier.h>
#include <sofa/collisionAlgorithm/geometry/TriangleGeometry.h>
#include <sofa/collisionAlgorithm/element/TriangleElement.h>

namespace sofa
{

namespace collisionAlgorithm
{

template<>
defaulttype::Vector3 PhongNormalHandler<TriangleElement>::internalGetNormal(const TriangleElement* element, const double* fact) const
{
    const std::vector<defaulttype::Vector3> & normals = element->geometry()->pointNormals();
    return normals[element->pointIDs()[0]] * fact[0] +
           normals[element->pointIDs()[1]] * fact[1] +
           normals[element->pointIDs()[2]] * fact[2];
}

}

}
