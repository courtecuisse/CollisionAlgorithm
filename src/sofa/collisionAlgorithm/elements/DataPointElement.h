#pragma once

#include <sofa/collisionAlgorithm/elements/DataElement.h>
#include <sofa/collisionAlgorithm/proximity/PointProximity.h>
#include <sofa/core/objectmodel/BaseObject.h>

namespace sofa
{

namespace collisionAlgorithm
{

template<class GEOMETRY>
class DataPointContainer : public DataElementContainer<GEOMETRY, defaulttype::Vector3, PointProximity> {
public:

    typedef PointProximity PROXIMITYDATA;
    typedef DataPointContainer<GEOMETRY> CONTAINER;
    typedef DataElementContainer<GEOMETRY, defaulttype::Vector3, PROXIMITYDATA> Inherit;

    typedef typename GEOMETRY::TDataTypes DataTypes;
    typedef typename DataTypes::VecCoord VecCoord;
    typedef typename DataTypes::Coord Coord;
    typedef typename DataTypes::Real Real;
    typedef typename DataTypes::VecDeriv VecDeriv;
    typedef typename DataTypes::MatrixDeriv MatrixDeriv;
    typedef typename MatrixDeriv::RowIterator MatrixDerivRowIterator;
    typedef core::objectmodel::Data< VecCoord >        DataVecCoord;
    typedef core::objectmodel::Data< VecDeriv >        DataVecDeriv;
    typedef core::objectmodel::Data< MatrixDeriv >     DataMatrixDeriv;
    typedef sofa::core::behavior::MechanicalState<DataTypes> State;
    typedef sofa::core::topology::BaseMeshTopology::Triangle Triangle;

    Data<double> d_drawRadius;

    explicit DataPointContainer(const typename Inherit::InitData& init)
    : Inherit(init)
    , d_drawRadius(dynamic_cast<sofa::core::objectmodel::BaseObject*>(init.owner)->initData(&d_drawRadius, (double) 1.0, "drawPointRadius", "radius of drawing")) {}

    defaulttype::BoundingBox getBBox(unsigned eid) const override {
        const helper::ReadAccessor<Data <VecCoord> >& x = this->getState()->read(core::VecCoordId::position());
        defaulttype::BoundingBox bbox;
        bbox.include(x[eid]);
        return bbox;
    }

    void draw(const core::visual::VisualParams *vparams, const defaulttype::Vector4 & color) override {
        Inherit::draw(vparams,color);

        if (! vparams->displayFlags().getShowCollisionModels()) return;
        if (color[3] == 0.0) return;
        if (this->d_drawRadius.getValue() == 0.0) return;

        const helper::ReadAccessor<DataVecCoord> & pos = this->getState()->read(core::VecCoordId::position());

        glColor4f(color[0],color[1],color[2],color[3]);

        for(auto it=this->begin();it!=this->end();it++) {
            vparams->drawTool()->drawSphere(pos[it->id()],d_drawRadius.getValue());
        }
    }

    inline PointProximity center(unsigned eid) const override {
        return PointProximity(eid);
    }

    inline PointProximity project(unsigned eid,const defaulttype::Vector3 & /*P*/) const override {
        return PointProximity(eid);
    }

    defaulttype::Vector3 getPosition(const PointProximity & data, core::VecCoordId v) const override {
        const helper::ReadAccessor<DataVecCoord> & pos = this->getState()->read(v);
        return pos[data.m_eid];
    }

    defaulttype::Vector3 getNormal(const PointProximity & /*data*/) const override {
        return defaulttype::Vector3();
    }

};


}

}
