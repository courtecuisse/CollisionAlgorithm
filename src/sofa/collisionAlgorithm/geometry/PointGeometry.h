#pragma once

#include <sofa/collisionAlgorithm/BaseGeometry.h>
#include <sofa/collisionAlgorithm/iterators/DefaultElementIterator.h>
#include <sofa/collisionAlgorithm/proximity/PointProximity.h>

namespace sofa {

namespace collisionAlgorithm {

template<class DataTypes>
class PointGeometry : public TBaseGeometry<DataTypes, PointProximity> {
public:
    typedef DataTypes TDataTypes;
    typedef PointProximity TPROXIMITYDATA;
    typedef PointGeometry<DataTypes> GEOMETRY;
    typedef TBaseGeometry<DataTypes, TPROXIMITYDATA> Inherit;
    typedef typename DataTypes::VecCoord VecCoord;
    typedef core::objectmodel::Data< VecCoord >        DataVecCoord;
    typedef typename DataTypes::MatrixDeriv MatrixDeriv;
    typedef typename MatrixDeriv::RowIterator MatrixDerivRowIterator;

    SOFA_CLASS(GEOMETRY,Inherit);

    Data<double> d_drawRadius;

    PointGeometry()
    : d_drawRadius(initData(&d_drawRadius, (double) 1.0, "drawPointRadius", "radius of drawing")) {}

    inline BaseElementIterator::UPtr begin(unsigned eid = 0) override {
        return DefaultElementIterator<GEOMETRY>::create(this, eid);
    }

    unsigned end() const {
        return this->l_state->getSize();
    }

    void draw(const core::visual::VisualParams *vparams) override {
        if (! vparams->displayFlags().getShowCollisionModels()) return;
        const defaulttype::Vector4 & color = this->d_color.getValue();
        if (color[3] == 0.0) return;
        if (d_drawRadius.getValue() == 0.0) return;

        const helper::ReadAccessor<DataVecCoord> & pos = this->getState()->read(core::VecCoordId::position());

        glColor4f(color[0],color[1],color[2],color[3]);

        for(auto it=this->begin();it!=this->end();it++) {
            vparams->drawTool()->drawSphere(pos[it->id()],d_drawRadius.getValue());
        }
    }

    inline defaulttype::BoundingBox getBBox(unsigned eid) const {
        const helper::ReadAccessor<DataVecCoord>& x = this->getState()->read(core::VecCoordId::position());
        defaulttype::BoundingBox bbox;
        bbox.include(x[eid]);
        return bbox;
    }

    inline PointProximity center(unsigned eid) const {
        return PointProximity(eid);
    }

    inline PointProximity project(unsigned eid,const defaulttype::Vector3 & /*P*/) const {
        return PointProximity(eid);
    }

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
