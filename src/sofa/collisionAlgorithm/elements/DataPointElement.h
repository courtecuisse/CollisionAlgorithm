#pragma once

#include <sofa/collisionAlgorithm/elements/DataBaseContainer.h>
#include <sofa/collisionAlgorithm/proximity/PointProximity.h>
#include <sofa/core/objectmodel/BaseObject.h>

namespace sofa
{

namespace collisionAlgorithm
{

template<class GEOMETRY>
class DataPointContainer : public DataBaseContainer<GEOMETRY, defaulttype::Vector3, PointProximity> {
public:

    typedef defaulttype::Vector3 ELEMENT;
    typedef PointProximity PROXIMITYDATA;
    typedef typename GEOMETRY::TDataTypes DataTypes;
    typedef typename DataTypes::Coord Coord;
    typedef typename DataTypes::VecCoord VecCoord;
    typedef core::objectmodel::Data< VecCoord >        DataVecCoord;
    typedef DataPointContainer<GEOMETRY> CONTAINER;
    typedef DataBaseContainer<GEOMETRY, ELEMENT, PROXIMITYDATA> Inherit;

    Data<double> d_drawRadius;

    DataPointContainer(const typename CONTAINER::InitData& init)
    : Inherit(init)
    , d_drawRadius(dynamic_cast<sofa::core::objectmodel::BaseObject*>(init.owner)->initData(&d_drawRadius, (double) 1.0, "drawPointRadius", "radius of drawing")) {}

    inline void draw(const core::visual::VisualParams *vparams) {
        const defaulttype::Vector4 & color = this->m_geometry->d_color.getValue();
        //if (! vparams->displayFlags().getShowCollisionModels()) return;
        if (color[3] == 0.0) return;
        if (this->d_drawRadius.getValue() == 0.0) return;

        const helper::ReadAccessor<DataVecCoord> & pos = this->getState()->read(core::VecCoordId::position());

        glColor4f(color[0],color[1],color[2],color[3]);

        for(auto it=this->begin();it!=this->end();it++) {
            vparams->drawTool()->drawSphere(pos[it->id()],d_drawRadius.getValue());
        }
    }

    virtual defaulttype::BoundingBox getBBox(unsigned eid) const {
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
