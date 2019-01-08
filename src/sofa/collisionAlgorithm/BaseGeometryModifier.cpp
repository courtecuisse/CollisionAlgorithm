#include <sofa/collisionAlgorithm/BaseGeometryModifier.h>
#include <sofa/collisionAlgorithm/element/TriangleElement.h>
#include <sofa/core/ObjectFactory.h>
#include <sofa/helper/types/RGBAColor.h>

namespace sofa
{

namespace collisionAlgorithm
{

void SofaBaseNormalHandler::draw(const sofa::core::visual::VisualParams* vparams)
{
    if(!vparams->displayFlags().getShowNormals())
        return;

    double fact[16];
    std::fill_n(fact, 16, 1.0); // suppose that element will never be bigger than 16 subparts...
    for(size_t i = 0 ; i<m_geometry->getNbElements() ; i++)
    {
        const BaseElement* element = m_geometry->getElement(i);
        const defaulttype::Vector3& normal = this->getNormal(i,fact);
        // center
        defaulttype::Vector3 center;// = element->getControlPoint(-1)->getPosition(); //supposed to give the center but seems not...
        for(size_t p = 0 ; p<element->getNbControlPoints() ; p++)
            center += element->getControlPoint(p)->getPosition();
        center /= element->getNbControlPoints();

        vparams->drawTool()->drawArrow(center, center+normal, 0.1f, helper::types::RGBAColor::gray());
    }

}

template<>
defaulttype::Vector3 SofaFlatNormalHandler<TriangleGeometry>::getNormal(const size_t id, const double* ) const
{
    return static_cast<const TriangleGeometry*>(m_geometry)->triangleInfo(id).tn;
}

template<>
defaulttype::Vector3 SofaFlatNormalHandler<PointGeometry>::getNormal(const size_t id, const double* ) const
{
    ///TODO !
    return defaulttype::Vector3(0,0,1);
}

int FlatNormalHandlerClass = core::RegisterObject("FlatNormalHandler")
.add< SofaFlatNormalHandler<TriangleGeometry> >()
.add< SofaFlatNormalHandler<PointGeometry> >()
.addAlias("FlatNormalHandler");



}

}
