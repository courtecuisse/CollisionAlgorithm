#pragma once

#include <sofa/collisionAlgorithm/BaseGeometry.h>
#include <sofa/collisionAlgorithm/iterators/DefaultElementIterator.h>
#include <sofa/collisionAlgorithm/proximity/PointProximity.h>

namespace sofa {

namespace collisionAlgorithm {

template<class DataTypes>
class PointGeometry : public TBaseGeometry<DataTypes,PointProximity> {
public:
    typedef DataTypes TDataTypes;
    typedef PointGeometry<DataTypes> GEOMETRY;
    typedef TBaseGeometry<DataTypes,PointProximity> Inherit;
    typedef BaseProximity::Index Index;
    typedef typename Inherit::PROXIMITYDATA PROXIMITYDATA;
    typedef typename DataTypes::Coord Coord;
    typedef typename DataTypes::VecCoord VecCoord;
    typedef core::objectmodel::Data< VecCoord >        DataVecCoord;
    typedef typename DataTypes::MatrixDeriv MatrixDeriv;
    typedef typename MatrixDeriv::RowIterator MatrixDerivRowIterator;

    SOFA_CLASS(GEOMETRY,Inherit);

    Data<double> d_drawRadius;

    PointGeometry()
    : d_drawRadius(initData(&d_drawRadius, (double) 1.0, "drawRadius", "radius of drawing")) {}

    inline BaseElementIterator::UPtr begin(Index eid = 0) const override {
        const helper::ReadAccessor<DataVecCoord> & pos = this->l_state->read(core::VecCoordId::position());
        return DefaultElementIterator<PROXIMITYDATA>::create(this, pos.ref(), eid);
    }

    void draw(const core::visual::VisualParams *vparams) override {
        this->drawNormals(vparams);

//        if (! vparams->displayFlags().getShowCollisionModels()) return;
        if (! vparams->displayFlags().getShowCollisionModels()) {
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

    inline PROXIMITYDATA createProximity(Index eid, CONTROL_POINT pid = CONTROL_DEFAULT) const {
        return PROXIMITYDATA::create(eid,pid);
    }

    //do not change the dataProximity.
    inline PROXIMITYDATA project(const defaulttype::Vector3 & /*Q*/, Index eid) const {
        return PROXIMITYDATA(eid);
    }

    inline defaulttype::Vector3 getPosition(const PROXIMITYDATA & data, core::VecCoordId v) const {
        const helper::ReadAccessor<DataVecCoord> & pos = this->getState()->read(v);
        return pos[data.m_eid];
    }

    virtual defaulttype::Vector3 computeNormal(const PROXIMITYDATA & data) const override {
        return defaulttype::Vector3();
    }
};

}

}
