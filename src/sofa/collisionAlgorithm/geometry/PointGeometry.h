#pragma once

#include <sofa/collisionAlgorithm/BaseGeometry.h>
#include <sofa/collisionAlgorithm/iterators/DefaultElementIterator.h>
#include <sofa/collisionAlgorithm/proximity/PointProximity.h>

namespace sofa
{

namespace collisionAlgorithm
{

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
        return DefaultElementIterator<GEOMETRY >::create(this, this->getState()->getSize(), eid);
    }

    inline BaseProximity::SPtr project(unsigned pid, const defaulttype::Vector3 & ) const {
        return BaseProximity::create<PointProximity<GEOMETRY> >(this,pid);
    }

    inline BaseProximity::SPtr center(unsigned pid) const {
        return BaseProximity::create<PointProximity<GEOMETRY> >(this,pid);
    }

    inline defaulttype::BoundingBox getBBox(unsigned pid) const {
        const helper::ReadAccessor<Data <VecCoord> >& x = this->getState()->read(core::VecCoordId::position());
        defaulttype::BoundingBox bbox;
        bbox.include(x[pid]);
        return bbox;
    }

    void draw(const core::visual::VisualParams *vparams) {
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
