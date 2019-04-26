#pragma once

#include <sofa/collisionAlgorithm/BaseGeometry.h>
#include <sofa/collisionAlgorithm/proximity/EdgeProximity.h>

namespace sofa {

namespace collisionAlgorithm {

template<class DataTypes>
class EdgeGeometry : public TBaseGeometry<DataTypes> {
public:
    typedef DataTypes TDataTypes;
    typedef EdgeProximity TPROXIMITYDATA;
    typedef TBaseGeometry<DataTypes> Inherit;
    typedef EdgeGeometry<DataTypes> GEOMETRY;
    typedef typename DataTypes::VecCoord VecCoord;
    typedef core::objectmodel::Data< VecCoord >        DataVecCoord;
    typedef typename DataTypes::MatrixDeriv MatrixDeriv;
    typedef typename MatrixDeriv::RowIterator MatrixDerivRowIterator;

    SOFA_CLASS(GEOMETRY,Inherit);

    core::objectmodel::SingleLink<GEOMETRY,core::topology::BaseMeshTopology,BaseLink::FLAG_STRONGLINK|BaseLink::FLAG_STOREPATH> l_topology;

    EdgeGeometry()
    : l_topology(initLink("topology", "link to topology")) {
        l_topology.setPath("@.");
    }

    inline BaseElementIterator::UPtr begin(unsigned eid = 0) override {
        return DefaultElementIterator<GEOMETRY>::create(this, eid);
    }

    unsigned end() const {
        return this->l_topology->getNbEdges();
    }

    inline defaulttype::BoundingBox getBBox(unsigned eid) const {
        const core::topology::BaseMeshTopology::Edge & edge = this->l_topology->getEdge(eid);
        const helper::ReadAccessor<Data <VecCoord> >& x = this->getState()->read(core::VecCoordId::position());
        defaulttype::BoundingBox bbox;
        bbox.include(x[edge[0]]);
        bbox.include(x[edge[1]]);
        return bbox;
    }

    inline EdgeProximity center(unsigned eid) const {
        const core::topology::BaseMeshTopology::Edge & edge = this->l_topology->getEdge(eid);

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
        sofa::core::topology::BaseMeshTopology::Edge edge = this->l_topology->getEdge(eid);

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
        this->drawNormals(vparams);

        if (! vparams->displayFlags().getShowCollisionModels()) return;
        const defaulttype::Vector4 & color = this->d_color.getValue();
        if (color[3] == 0.0) return;

        glDisable(GL_LIGHTING);

        glBegin(GL_LINES);
        glColor4dv(color.data());
        const helper::ReadAccessor<DataVecCoord> & pos = this->getState()->read(core::VecCoordId::position());

        for (auto it=this->begin();it!=this->end();it++) {
            const sofa::core::topology::BaseMeshTopology::Edge & edge = this->l_topology->getEdge(it->id());

            glVertex3dv(pos[edge[0]].data());
            glVertex3dv(pos[edge[1]].data());
        }
        glEnd();
    }

};

}

}
