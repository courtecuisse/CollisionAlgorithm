#pragma once

#include <sofa/collisionAlgorithm/BaseGeometry.h>
#include <sofa/collisionAlgorithm/iterators/DefaultElementIterator.h>
#include <sofa/collisionAlgorithm/proximity/PointProximity.h>

namespace sofa
{

namespace collisionAlgorithm
{

template<class GEOMETRY>
class PointElement : public BaseElement {
public:
    typedef GEOMETRY TGeometry;
    typedef typename GEOMETRY::TDataTypes DataTypes;
    typedef typename DataTypes::VecCoord VecCoord;
    typedef Data<VecCoord> DataVecCoord;

    PointElement(unsigned id,const GEOMETRY * geo) : m_id(id), m_geo(geo) {}

    inline BaseProximity::SPtr project(const defaulttype::Vector3 & /*P*/) const {
        return BaseProximity::create<PointProximity<GEOMETRY> >(m_geo,m_id);
    }

    inline BaseProximity::SPtr center() const {
        return BaseProximity::create<PointProximity<GEOMETRY> >(m_geo,m_id);
    }

    inline defaulttype::BoundingBox getBBox() const {
        const helper::ReadAccessor<Data <VecCoord> >& x = m_geo->getState()->read(core::VecCoordId::position());
        defaulttype::BoundingBox bbox;
        bbox.include(x[m_id]);
        return bbox;
    }

protected:
    unsigned m_id;
    const GEOMETRY * m_geo;
};


template<class DataTypes>
class PointGeometry : public TBaseGeometry<DataTypes> {
public:
    typedef TBaseGeometry<DataTypes> Inherit;
    typedef PointGeometry<DataTypes> GEOMETRY;
    typedef DataTypes TDataTypes;
    typedef typename DataTypes::Coord Coord;
    typedef typename DataTypes::Deriv Deriv;
    typedef typename DataTypes::VecCoord VecCoord;
    typedef Data<VecCoord> DataVecCoord;

    SOFA_CLASS(GEOMETRY,Inherit);

    virtual BaseElementIterator::UPtr begin(unsigned eid = 0) const {
        return DefaultElementIterator<PointElement<GEOMETRY> >::create(this, this->getState()->getSize(), eid);
    }

    virtual void draw(const core::visual::VisualParams *vparams) {
        Inherit::draw(vparams);

        if (! vparams->displayFlags().getShowCollisionModels())
            return;

        if (this->d_color.getValue()[3] == 0.0)
            return;

        glDisable(GL_LIGHTING);

    //    for(ElementIterator it=elementIterator();!it.end();it.next()) {
    //        m_elements[i]->draw(vparams);
    //    }
    }

};

}

}
