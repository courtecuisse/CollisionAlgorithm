#pragma once

#include <sofa/collisionAlgorithm/elements/DataBaseContainer.h>
#include <sofa/collisionAlgorithm/proximity/EdgeProximity.h>

namespace sofa
{

namespace collisionAlgorithm
{

template<class GEOMETRY>
class DataEdgeContainer : public DataBaseContainer<GEOMETRY, sofa::core::topology::BaseMeshTopology::Edge, EdgeProximity> {
public:

    typedef sofa::core::topology::BaseMeshTopology::Edge ELEMENT;
    typedef EdgeProximity PROXIMITYDATA;
    typedef typename GEOMETRY::TDataTypes DataTypes;
    typedef typename DataTypes::Coord Coord;
    typedef typename DataTypes::VecCoord VecCoord;
    typedef core::objectmodel::Data< VecCoord >        DataVecCoord;
    typedef DataEdgeContainer<GEOMETRY> CONTAINER;
    typedef DataBaseContainer<GEOMETRY, sofa::core::topology::BaseMeshTopology::Edge ,EdgeProximity> Inherit;

    DataEdgeContainer(const typename CONTAINER::InitData& init)
    : Inherit(init) {}

    inline defaulttype::BoundingBox getBBox(unsigned eid) const {
        const core::topology::BaseMeshTopology::Edge & edge = this->element(eid);
        const helper::ReadAccessor<Data <VecCoord> >& x = this->getState()->read(core::VecCoordId::position());
        defaulttype::BoundingBox bbox;
        bbox.include(x[edge[0]]);
        bbox.include(x[edge[1]]);
        return bbox;
    }

    inline EdgeProximity center(unsigned eid) const {
        const core::topology::BaseMeshTopology::Edge & edge = this->element(eid);

        return EdgeProximity(eid,edge[0],edge[1],0.5,0.5);
    }

    inline defaulttype::Vector3 getPosition(const EdgeProximity & data, core::VecCoordId v = core::VecCoordId::position()) const {
        const helper::ReadAccessor<DataVecCoord> & pos = this->getState()->read(v);

        return pos[data.m_p0] * data.m_f0 +
               pos[data.m_p1] * data.m_f1;
    }

    inline defaulttype::Vector3 getNormal(const EdgeProximity & data) const {
        const helper::ReadAccessor<DataVecCoord> & pos = this->getState()->read(core::VecCoordId::position());
        return (pos[data.m_p1] - pos[data.m_p0]).normalized();
    }

    inline EdgeProximity project(unsigned eid, const defaulttype::Vector3 & P) const {
        sofa::core::topology::BaseMeshTopology::Edge edge = this->element(eid);

        const helper::ReadAccessor<Data <VecCoord> >& x = this->getState()->read(core::VecCoordId::position());

        const defaulttype::Vector3 & E1 = x[edge[0]];
        const defaulttype::Vector3 & E2 = x[edge[1]];

        double fact_u;
        double fact_v;

        defaulttype::Vector3 v = E2 - E1;
        fact_v = dot (P - E1,v) / dot(v,v);

        if (fact_v<0.0) fact_v = 0.0;
        else if (fact_v>1.0) fact_v = 1.0;

        fact_u = 1.0-fact_v;

        return EdgeProximity(eid, edge[0],edge[1], fact_u,fact_v);
    }

    inline void draw(const core::visual::VisualParams * vparams) {
        if (! vparams->displayFlags().getShowCollisionModels()) return;
        const defaulttype::Vector4 & color = this->m_geometry->d_color.getValue();
        if (color[3] == 0.0) return;

        glDisable(GL_LIGHTING);

        glBegin(GL_LINES);
        glColor4dv(color.data());
        const helper::ReadAccessor<DataVecCoord> & pos = this->getState()->read(core::VecCoordId::position());

        for (auto it=this->begin();it!=this->end();it++) {
            const sofa::core::topology::BaseMeshTopology::Edge & edge = this->element(it->id());

            glVertex3dv(pos[edge[0]].data());
            glVertex3dv(pos[edge[1]].data());
        }
        glEnd();
    }

    inline BaseElementIterator::UPtr begin(unsigned eid = 0) override {
        return DefaultElementIterator<CONTAINER>::create(this, eid);
    }

};


}

}
