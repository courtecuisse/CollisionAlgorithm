#include <sofa/collisionAlgorithm/BaseGeometryModifier.h>
#include <sofa/collisionAlgorithm/element/TriangleElement.h>
#include <sofa/core/ObjectFactory.h>
namespace sofa
{

namespace collisionAlgorithm
{

template<class TGeometry>
defaulttype::Vector3 FlatNormalHandler<TGeometry>::internalGetNormal(const BaseGeometry*, const size_t, const double* ) const
{
    msg_error("FlatNormalHandler") << "Geometry does not implement Flat Normal";
    return defaulttype::Vector3();
}

template<>
defaulttype::Vector3 FlatNormalHandler<TriangleGeometry>::internalGetNormal(const BaseGeometry* geometry, const size_t id, const double* ) const
{
    return static_cast<const TriangleGeometry*>(geometry)->triangleInfo(id).tn;
}

int FlatNormalHandlerClass = core::RegisterObject("FlatNormalHandler")
.add< SofaFlatNormalHandler >();

SofaFlatNormalHandler::SofaFlatNormalHandler()
: SofaBaseNormalHandler()
{
    if(dynamic_cast<TriangleGeometry*>(l_geometry.get()))
    {
        m_impl = new FlatNormalHandler<TriangleGeometry>();
    }

}


}

}
