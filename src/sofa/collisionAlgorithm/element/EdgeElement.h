#pragma once

#include <sofa/collisionAlgorithm/geometry/EdgeGeometry.h>
#include <sofa/collisionAlgorithm/proximity/EdgeProximity.h>

namespace sofa
{

namespace collisionAlgorithm
{

//Internal iterator of elements
template<class DataTypes>
class EdgeElementIterator : public DefaultElement {
public:
    typedef sofa::core::behavior::MechanicalState<DataTypes> State;
    typedef typename DataTypes::Coord Coord;
    typedef typename DataTypes::VecCoord VecCoord;
    typedef Data<VecCoord> DataVecCoord;

    EdgeElementIterator(const EdgeGeometry<DataTypes> * geo) : m_geometry(geo) {
        m_state = m_geometry->l_state.get();
    }

    BaseProximity::SPtr project(const defaulttype::Vector3 & P) const {
        const core::topology::BaseMeshTopology::Edge edge = m_geometry->d_edges.getValue()[id()];

        const helper::ReadAccessor<Data <VecCoord> >& x = *m_state->read(core::VecCoordId::position());

        double fact_u;
        double fact_v;

        Coord v = x[edge[1]] - x[edge[0]];
        fact_v = dot (P - x[edge[0]],v) / dot (v,v);

        if (fact_v<0.0) fact_v = 0.0;
        else if (fact_v>1.0) fact_v = 1.0;

        fact_u = 1.0-fact_v;

        return BaseProximity::SPtr(new EdgeProximity<DataTypes>(edge[0],edge[1],fact_u,fact_v,m_state));
    }

    bool end(const BaseGeometry */*geo*/) const {
        return id() < m_geometry->d_edges.getValue().size();
    }

    virtual BaseProximity::SPtr center() const {
        const core::topology::BaseMeshTopology::Edge edge = m_geometry->d_edges.getValue()[id()];
        return BaseProximity::SPtr(new EdgeProximity<DataTypes>(edge[0],edge[1],0.5,0.5,m_state));
    }

    virtual defaulttype::BoundingBox getBBox() const {
        const core::topology::BaseMeshTopology::Edge edge = m_geometry->d_edges.getValue()[id()];
        const helper::ReadAccessor<Data <VecCoord> >& x = *m_state->read(core::VecCoordId::position());
        defaulttype::BoundingBox bbox;
        bbox.include(x[edge[0]]);
        bbox.include(x[edge[1]]);
        return bbox;
    }

    const EdgeGeometry<DataTypes> * m_geometry;
    State * m_state;
};


//    void draw(const core::visual::VisualParams * /*vparams*/) const override
//    {
//        glColor4dv(geometry()->d_color.getValue().data());

//        glBegin(GL_LINES);
//            glVertex3dv(getControlPoint(0)->getPosition().data());
//            glVertex3dv(getControlPoint(1)->getPosition().data());
//        glEnd();
//    }

//protected:
//    const EdgeGeometry* m_geometry;
//    size_t m_pid[2];
//    size_t m_eid;
//};

}

}
