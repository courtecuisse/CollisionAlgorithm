#pragma once

#include <sofa/collisionAlgorithm/BaseGeometry.h>
#include <sofa/collisionAlgorithm/iterators/DefaultElementIterator.h>
#include <sofa/collisionAlgorithm/operations/EdgeOperation.h>

namespace sofa {

namespace collisionAlgorithm {

template<class DataTypes>
class EdgeGeometry : public TBaseGeometry<DataTypes> {
public:
    typedef DataTypes TDataTypes;
    typedef TBaseGeometry<DataTypes,EdgeProximity> Inherit;
    typedef BaseProximity::Index Index;
    typedef typename Inherit::PROXIMITYDATA PROXIMITYDATA;
    typedef EdgeGeometry<DataTypes> GEOMETRY;
    typedef typename DataTypes::VecCoord VecCoord;
    typedef core::objectmodel::Data< VecCoord >        DataVecCoord;
    typedef typename DataTypes::MatrixDeriv MatrixDeriv;
    typedef typename MatrixDeriv::RowIterator MatrixDerivRowIterator;
    typedef sofa::topology::Edge Edge;

    SOFA_CLASS(GEOMETRY,Inherit);

    core::objectmodel::SingleLink<GEOMETRY,core::topology::BaseMeshTopology,BaseLink::FLAG_STRONGLINK|BaseLink::FLAG_STOREPATH> l_topology;

    EdgeGeometry()
    : l_topology(initLink("topology", "link to topology")) {
        l_topology.setPath("@.");
    }

    template<class DataTypes>
    static BaseElement::SPtr createElement(sofa::core::behavior::MechanicalState<DataTypes> * state, core::topology::BaseMeshTopology::Edge edge) {
        auto p0 = PointOperation::createProximity(state,edge[0]);
        auto p1 = PointOperation::createProximity(state,edge[1]);

        return BaseElement::SPtr(new EdgeElement(p0,p1));
    }


    inline BaseElementIterator::SPtr begin(Index eid = 0) const override {
        return DefaultElementIterator<PROXIMITYDATA>::create(this,this->l_topology->getEdges(), eid);
    }

    inline const sofa::topology::Edge getEdge(Index eid) const {
        return this->l_topology->getEdge(eid);
    }

    inline sofa::type::Vector3 getPosition(const PROXIMITYDATA & data, core::VecCoordId v = core::VecCoordId::position()) const {
        const helper::ReadAccessor<DataVecCoord> & pos = this->getState()->read(v);

        return pos[data.m_p0] * data.m_f0 +
               pos[data.m_p1] * data.m_f1;
    }

    inline PROXIMITYDATA createProximity(Index eid, CONTROL_POINT c = CONTROL_DEFAULT) const {
        return PROXIMITYDATA::create(eid, getEdge(eid), c);
    }

    inline PROXIMITYDATA project(const sofa::type::Vector3 & P, Index eid) const {
        auto edge = getEdge(eid);

        const helper::ReadAccessor<Data <VecCoord> >& x = this->getState()->read(core::VecCoordId::position());

        const sofa::type::Vector3 & E1 = x[edge[0]];
        const sofa::type::Vector3 & E2 = x[edge[1]];

        double fact_u;
        double fact_v;

        toolBox::projectOnEdge(P,E1,E2,fact_u,fact_v);

        return PROXIMITYDATA(eid, edge[0], edge[1], fact_u,fact_v);
    }

    virtual sofa::type::Vector3 computeNormal(const PROXIMITYDATA & /*data*/) const override {
        return sofa::type::Vector3();
    }

    inline void draw(const core::visual::VisualParams * vparams) {
        this->drawNormals(vparams);

//        if (! vparams->displayFlags().getShowCollisionModels()) return;
        if (! vparams->displayFlags().getShowCollisionModels()) return ;
        const sofa::type::RGBAColor & color = this->d_color.getValue();
        if (color[3] == 0.0) return;

        glDisable(GL_LIGHTING);

        glBegin(GL_LINES);
        glColor4fv(color.data());
        const helper::ReadAccessor<DataVecCoord> & pos = this->getState()->read(core::VecCoordId::position());

        for (auto it=this->begin();it!=this->end();it++) {
            const sofa::topology::Edge & edge = this->l_topology->getEdge(it->id());

            glColor4f(1.0,0.0,0.0,1.0);
            glVertex3dv(pos[edge[0]].data());
            glColor4f(0.0,0.0,1.0,1.0);
            glVertex3dv(pos[edge[1]].data());
        }
        glEnd();
    }

};

}

}
