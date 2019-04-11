#pragma once

#include <sofa/collisionAlgorithm/BaseGeometry.h>
#include <sofa/collisionAlgorithm/iterators/DefaultElementIterator.h>
#include <sofa/collisionAlgorithm/proximity/PointProximity.h>
#include <sofa/collisionAlgorithm/elements/PointElement.h>

namespace sofa {

namespace collisionAlgorithm {

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

    Data<double> d_drawRadius;

    PointGeometry()
    : d_drawRadius(initData(&d_drawRadius, 1.0, "radius", "radius of drawing")) {}

    virtual BaseElementIterator::UPtr getElementIterator(unsigned eid = 0) const {
        return DefaultElementIterator<PointElement<GEOMETRY> >::create(this, this->getState()->getSize(), eid);
    }

    virtual void draw(const core::visual::VisualParams *vparams) {
        Inherit::draw(vparams);

        if (! vparams->displayFlags().getShowCollisionModels()) return;
        if (this->d_color.getValue()[3] == 0.0) return;
        if (this->d_drawRadius.getValue() == 0.0) return;

        const helper::ReadAccessor<DataVecCoord> & pos = this->l_state->read(core::VecCoordId::position());

        defaulttype::Vector4 color = this->d_color.getValue();
        glColor4f(color[0],color[1],color[2],color[3]);

        for(auto it=this->begin();it!=this->end();it++) {
            vparams->drawTool()->drawSphere(pos[it->id()],d_drawRadius.getValue());
        }
    }

};

}

}
