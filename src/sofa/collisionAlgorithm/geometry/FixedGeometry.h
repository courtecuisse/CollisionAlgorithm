#pragma once

#include <sofa/collisionAlgorithm/BaseGeometry.h>
#include <sofa/collisionAlgorithm/iterators/DefaultElementIterator.h>
#include <sofa/collisionAlgorithm/proximity/FixedProximity.h>

namespace sofa {

namespace collisionAlgorithm {

template<class DataTypes>
class FixedGeometry : public TBaseGeometry<DataTypes> {
public:
    typedef DataTypes TDataTypes;
    typedef FixedGeometry<DataTypes> GEOMETRY;
    typedef TBaseGeometry<DataTypes> Inherit;
    typedef typename DataTypes::Coord Coord;
    typedef typename DataTypes::VecCoord VecCoord;
    typedef core::objectmodel::Data< VecCoord >        DataVecCoord;
    typedef typename DataTypes::MatrixDeriv MatrixDeriv;
    typedef typename MatrixDeriv::RowIterator MatrixDerivRowIterator;

    SOFA_CLASS(GEOMETRY,Inherit);

    Data<double> d_drawRadius;
    Data<sofa::helper::vector<defaulttype::Vector3> > d_normals;

    FixedGeometry()
    : d_drawRadius(initData(&d_drawRadius, (double) 1.0, "drawRadius", "radius of drawing"))
    , d_normals(initData(&d_normals,sofa::helper::vector<defaulttype::Vector3>(), "normals","normals"))
    {}

    inline BaseElementIterator::UPtr begin(unsigned eid = 0) const override {
        const helper::ReadAccessor<DataVecCoord> & pos = this->l_state->read(core::VecCoordId::position());
        return DefaultElementIterator<FixedProximity>::create(this, pos.ref(), eid);
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

    inline defaulttype::BoundingBox getBBox(const Coord & p) const {
        defaulttype::BoundingBox bbox;
        bbox.include(p);
        return bbox;
    }

    inline FixedProximity center(unsigned eid, const Coord & /*p*/) const {
        const helper::ReadAccessor<DataVecCoord> & pos = this->getState()->read(core::VecCoordId::position());
        if(d_normals.getValue().size()>eid)
            return FixedProximity(pos[eid],d_normals.getValue()[eid]);
        else
            return FixedProximity(pos[eid]);
    }

    //do not change the dataProximity.
    inline FixedProximity project(unsigned pid, const Coord & /*P*/,const defaulttype::Vector3 & /*Q*/) const {
        const helper::ReadAccessor<DataVecCoord> & pos = this->getState()->read(core::VecCoordId::position());
        if(d_normals.getValue().size()>pid)
            return FixedProximity(pos[pid],d_normals.getValue()[pid]);
        else
            return FixedProximity(pos[pid]);
    }

    inline defaulttype::Vector3 getPosition(const FixedProximity & data, core::VecCoordId v) const {
        return data.getPosition(v);
    }

    inline defaulttype::Vector3 getNormal(const FixedProximity & data) const {
        return data.m_normal;
    }

};

}

}
