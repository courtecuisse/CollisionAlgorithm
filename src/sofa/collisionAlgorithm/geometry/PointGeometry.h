#pragma once

#include <sofa/collisionAlgorithm/BaseGeometry.h>
#include <sofa/collisionAlgorithm/iterators/DefaultElementIterator.h>
#include <sofa/collisionAlgorithm/proximity/PointProximity.h>

namespace sofa {

namespace collisionAlgorithm {

template<class DataTypes>
class PointGeometry : public TBaseGeometry<DataTypes> {
public:
    typedef DataTypes TDataTypes;
    typedef PointGeometry<DataTypes> GEOMETRY;
    typedef TBaseGeometry<DataTypes> Inherit;
    typedef typename DataTypes::VecCoord VecCoord;
    typedef core::objectmodel::Data< VecCoord >        DataVecCoord;
    typedef typename DataTypes::MatrixDeriv MatrixDeriv;
    typedef typename MatrixDeriv::RowIterator MatrixDerivRowIterator;

    SOFA_CLASS(GEOMETRY,Inherit);

    Data<double> d_drawRadius;

    PointGeometry()
    : d_drawRadius(initData(&d_drawRadius, (double) 1.0, "radius", "radius of drawing")) {}

    inline BaseElementIterator::UPtr begin(unsigned eid = 0) override {
        return DefaultElementIterator<GEOMETRY, PointProximity>::create(this, this->l_state->getSize(), eid);
    }

    void draw(const core::visual::VisualParams *vparams) override {
        this->drawNormals(vparams);

//        if (! vparams->displayFlags().getShowCollisionModels()) return;
        if (! this->drawCollision.getValue() && ! vparams->displayFlags().getShowCollisionModels()) {
            return ;
        }
        const defaulttype::Vector4 & color = this->d_color.getValue();
        if (color[3] == 0.0) return;
        if (d_drawRadius.getValue() == 0.0) return;

        const helper::ReadAccessor<DataVecCoord> & pos = this->getState()->read(core::VecCoordId::position());

        glColor4f(color[0],color[1],color[2],color[3]);

        for(auto it=this->begin();it != this->end();it++) {
            vparams->drawTool()->drawSphere(pos[it->id()],d_drawRadius.getValue());
        }
    }

    //do not change the dataProximity.
    inline void project(PointProximity & /*data*/,const defaulttype::Vector3 & /*P*/) const {}

    inline defaulttype::Vector3 getPosition(const PointProximity & data, core::VecCoordId v) const {
        const helper::ReadAccessor<DataVecCoord> & pos = this->getState()->read(v);
        return pos[data.m_eid];
    }

    inline defaulttype::Vector3 getNormal(const PointProximity & /*data*/) const {
        return defaulttype::Vector3();
    }

};

}

}
