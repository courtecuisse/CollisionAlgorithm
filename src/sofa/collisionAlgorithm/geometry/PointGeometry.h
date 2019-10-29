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
    typedef typename DataTypes::Coord Coord;
    typedef typename DataTypes::VecCoord VecCoord;
    typedef core::objectmodel::Data< VecCoord >        DataVecCoord;
    typedef typename DataTypes::MatrixDeriv MatrixDeriv;
    typedef typename MatrixDeriv::RowIterator MatrixDerivRowIterator;

    SOFA_CLASS(GEOMETRY,Inherit);

    class PointNormalHandler : public sofa::core::objectmodel::BaseObject {
    public:
        SOFA_ABSTRACT_CLASS(PointNormalHandler, sofa::core::objectmodel::BaseObject);

        core::objectmodel::SingleLink<PointNormalHandler, GEOMETRY, BaseLink::FLAG_STRONGLINK|BaseLink::FLAG_STOREPATH> l_geometry;

        PointNormalHandler()
        : l_geometry(initLink("geometry", "link to the geometry")) {
            l_geometry.setPath("@.");
        }

        void init() override {
            if (l_geometry == NULL) return;
            l_geometry->m_normalHandler = this;
            l_geometry->addSlave(this);
        }

        virtual defaulttype::Vector3 computeNormal(const PointProximity & data) const = 0;
    };

    Data<double> d_drawRadius;

    PointGeometry()
    : d_drawRadius(initData(&d_drawRadius, (double) 1.0, "drawRadius", "radius of drawing")) {}

    //Check at bwd init if the normal handler is set else create a default one
    void bwdInit();

    inline BaseElementIterator::UPtr begin(unsigned eid = 0) const override {
        const helper::ReadAccessor<DataVecCoord> & pos = this->l_state->read(core::VecCoordId::position());
        return DefaultElementIterator<PointProximity>::create(this, pos.ref(), eid);
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

    inline defaulttype::BoundingBox getBBox(const Coord & p) const {
        defaulttype::BoundingBox bbox;
        bbox.include(p);
        return bbox;
    }

    inline PointProximity center(unsigned eid, const Coord & /*p*/) const {
        return PointProximity(eid);
    }

    //do not change the dataProximity.
    inline PointProximity project(unsigned pid, const Coord & /*P*/,const defaulttype::Vector3 & /*Q*/) const {
        return PointProximity(pid);
    }

    inline defaulttype::Vector3 getPosition(const PointProximity & data, core::VecCoordId v) const {
        const helper::ReadAccessor<DataVecCoord> & pos = this->getState()->read(v);
        return pos[data.m_eid];
    }

    inline defaulttype::Vector3 getNormal(const PointProximity & data) const {
        return m_normalHandler->computeNormal(data);
    }

protected:
    typename PointNormalHandler::SPtr m_normalHandler;
};

}

}
