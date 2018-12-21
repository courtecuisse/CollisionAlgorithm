#include <sofa/collisionAlgorithm/BaseGeometryModifier.h>
#include <sofa/collisionAlgorithm/element/TriangleElement.h>
#include <sofa/core/ObjectFactory.h>
namespace sofa
{

namespace collisionAlgorithm
{

template<class TElement>
defaulttype::Vector3 FlatNormalHandler<TElement>::internalGetNormal(const TElement*, const double* ) const
{
    msg_error("FlatNormalHandler") << "Element does not implement Phong Normal";
    return defaulttype::Vector3();
}

template<>
defaulttype::Vector3 FlatNormalHandler<TriangleElement>::internalGetNormal(const TriangleElement* element, const double* ) const
{
    return element->geometry()->triangleInfo(element->id()).tn;
}

int FlatNormalHandlerClass = core::RegisterObject("FlatNormalHandler")
.add< SofaFlatNormalHandler >();

SofaFlatNormalHandler::SofaFlatNormalHandler()
: SofaBaseNormalHandler()
{
    if(dynamic_cast<TriangleGeometry*>(l_geometry.get()))
    {
        m_impl = new FlatNormalHandler<TriangleElement>();
    }
}


}

}
