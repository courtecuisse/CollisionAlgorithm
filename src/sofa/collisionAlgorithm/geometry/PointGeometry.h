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

        const helper::ReadAccessor<DataVecCoord> & pos = this->l_state->read(core::VecCoordId::position());

        glBegin(GL_POINTS);
        for(auto it=begin();it!=this->end();it++) {
            glVertex3dv(pos[it->id()].data());
        }
        glEnd();
    }

};

}

}
