#pragma once

#include <sofa/collisionAlgorithm/BaseGeometry.h>
#include <sofa/collisionAlgorithm/iterators/DefaultElementIterator.h>
#include <sofa/collisionAlgorithm/proximity/EdgeProximity.h>

namespace sofa {

namespace collisionAlgorithm {

template<class DataTypes>
class EdgeGeometry : public TBaseGeometry<DataTypes> {
public:
    typedef DataTypes TDataTypes;
    typedef TBaseGeometry<DataTypes> Inherit;
    typedef EdgeGeometry<DataTypes> GEOMETRY;
    typedef typename DataTypes::VecCoord VecCoord;
    typedef core::objectmodel::Data< VecCoord >        DataVecCoord;
    typedef typename DataTypes::MatrixDeriv MatrixDeriv;
    typedef typename MatrixDeriv::RowIterator MatrixDerivRowIterator;
    typedef sofa::core::topology::BaseMeshTopology::Edge Edge;

    SOFA_CLASS(GEOMETRY,Inherit);

    class EdgeNormalHandler : public sofa::core::objectmodel::BaseObject {
    public:
        SOFA_ABSTRACT_CLASS(EdgeNormalHandler, sofa::core::objectmodel::BaseObject);

        core::objectmodel::SingleLink<EdgeNormalHandler, EdgeGeometry, BaseLink::FLAG_STRONGLINK|BaseLink::FLAG_STOREPATH> l_geometry;

        EdgeNormalHandler()
        : l_geometry(initLink("geometry", "link to the geometry")) {
            l_geometry.setPath("@.");
        }

        void init() override {
            if (l_geometry == NULL) return;
            l_geometry->m_normalHandler = this;
            l_geometry->addSlave(this);
        }

        virtual defaulttype::Vector3 computeNormal(const EdgeProximity & data) const = 0;
    };

    core::objectmodel::SingleLink<GEOMETRY,core::topology::BaseMeshTopology,BaseLink::FLAG_STRONGLINK|BaseLink::FLAG_STOREPATH> l_topology;

    EdgeGeometry()
    : l_topology(initLink("topology", "link to topology")) {
        l_topology.setPath("@.");
    }

    //Check at bwd init if the normal handler is set else create a default one
    void bwdInit();

    inline BaseElementIterator::UPtr begin(unsigned eid = 0) const override {
        return DefaultElementIterator<EdgeProximity>::create(this,this->l_topology->getEdges(), eid);
    }

    inline const sofa::core::topology::BaseMeshTopology::Edge getEdge(unsigned eid) const {
        return this->l_topology->getEdge(eid);
    }

    inline defaulttype::Vector3 getPosition(const EdgeProximity & data, core::VecCoordId v = core::VecCoordId::position()) const {
        const helper::ReadAccessor<DataVecCoord> & pos = this->getState()->read(v);

        return pos[data.m_p0] * data.m_f0 +
               pos[data.m_p1] * data.m_f1;
    }

    inline defaulttype::Vector3 getNormal(const EdgeProximity & data) const {
        return m_normalHandler->computeNormal(data);

    }

    inline defaulttype::BoundingBox getBBox(const Edge & edge) const {
        const helper::ReadAccessor<Data <VecCoord> >& x = this->getState()->read(core::VecCoordId::position());

        defaulttype::BoundingBox bbox;
        bbox.include(x[edge[0]]);
        bbox.include(x[edge[1]]);
        return bbox;
    }

    inline EdgeProximity center(unsigned eid, const Edge & edge) const {
        return EdgeProximity(eid, edge[0], edge[1], 0.5, 0.5);
    }

    inline EdgeProximity project(unsigned eid, const Edge & edge, const defaulttype::Vector3 & P) const {
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

        return EdgeProximity(eid, edge[0], edge[1], fact_u,fact_v);
    }

    inline void draw(const core::visual::VisualParams * vparams) {
        this->drawNormals(vparams);

//        if (! vparams->displayFlags().getShowCollisionModels()) return;
        if (! vparams->displayFlags().getShowCollisionModels()) return ;
        const defaulttype::Vector4 & color = this->d_color.getValue();
        if (color[3] == 0.0) return;

        glDisable(GL_LIGHTING);

        glBegin(GL_LINES);
        glColor4dv(color.data());
        const helper::ReadAccessor<DataVecCoord> & pos = this->getState()->read(core::VecCoordId::position());

        for (auto it=this->begin();it!=this->end();it++) {
            const sofa::core::topology::BaseMeshTopology::Edge & edge = this->l_topology->getEdge(it->id());

            glColor4f(1.0,0.0,0.0,1.0);
            glVertex3dv(pos[edge[0]].data());
            glColor4f(0.0,0.0,1.0,1.0);
            glVertex3dv(pos[edge[1]].data());
        }
        glEnd();
    }

protected:
    typename EdgeNormalHandler::SPtr m_normalHandler;

};

}

}
