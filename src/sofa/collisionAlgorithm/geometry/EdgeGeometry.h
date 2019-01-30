#pragma once

#include <sofa/collisionAlgorithm/geometry/PointGeometry.h>
#include <sofa/collisionAlgorithm/proximity/EdgeProximity.h>

namespace sofa {

namespace collisionAlgorithm {

template<class GEOMETRY>
class EdgeElement : public BaseElement {
public:
    typedef GEOMETRY TGeometry;
    typedef typename GEOMETRY::TDataTypes DataTypes;
    typedef typename DataTypes::VecCoord VecCoord;
    typedef Data<VecCoord> DataVecCoord;

    EdgeElement(unsigned id,const GEOMETRY * geo) : m_eid(id), m_geo(geo) {}

    inline BaseProximity::SPtr project(const defaulttype::Vector3 & P) const {
        core::topology::BaseMeshTopology::Edge edge;
        defaulttype::Vector2 factor;
        m_geo->project(m_eid, P, edge, factor);

        return BaseProximity::create<EdgeProximity<GEOMETRY> >(m_geo,m_eid, edge[0],edge[1],factor[0],factor[1]);
    }

    inline BaseProximity::SPtr center() const {
        const core::topology::BaseMeshTopology::Edge & edge = m_geo->getEdges()[m_eid];
        return BaseProximity::create<EdgeProximity<GEOMETRY> >(m_geo,m_eid, edge[0],edge[1],0.5,0.5);
    }

    inline defaulttype::BoundingBox getBBox() const {
        const core::topology::BaseMeshTopology::Edge & edge = m_geo->getEdges()[m_eid];
        const helper::ReadAccessor<Data <VecCoord> >& x = m_geo->getState()->read(core::VecCoordId::position());
        defaulttype::BoundingBox bbox;
        bbox.include(x[edge[0]]);
        bbox.include(x[edge[1]]);
        return bbox;
    }

protected:
    unsigned m_eid;
    const GEOMETRY * m_geo;
};

template<class DataTypes>
class EdgeGeometry : public TBaseGeometry<DataTypes> {
public:
    typedef DataTypes TDataTypes;
    typedef TBaseGeometry<DataTypes> Inherit;
    typedef EdgeGeometry<DataTypes> GEOMETRY;
    typedef typename DataTypes::Coord Coord;
    typedef typename DataTypes::VecCoord VecCoord;
    typedef Data<VecCoord> DataVecCoord;
    typedef sofa::core::topology::BaseMeshTopology::Edge Edge;
    typedef helper::vector<Edge> VecEdges;

    SOFA_CLASS(GEOMETRY,Inherit);

    Data<VecEdges> d_edges;

    EdgeGeometry()
    : d_edges(initData(&d_edges, "edges", "Vector of Edges")){}

    virtual BaseElementIterator::UPtr begin(unsigned eid = 0) const {
        return DefaultElementIterator<EdgeElement<GEOMETRY> >::create(this, d_edges.getValue().size(), eid);
    }

    inline const VecEdges & getEdges() const {
        return d_edges.getValue();
    }

    virtual void draw(const core::visual::VisualParams * vparams) {
        Inherit::draw(vparams);

        if (! vparams->displayFlags().getShowCollisionModels())
            return;

        if (this->d_color.getValue()[3] == 0.0)
            return;

        glDisable(GL_LIGHTING);

        glBegin(GL_LINES);
        glColor4dv(this->d_color.getValue().data());
        const helper::ReadAccessor<DataVecCoord> & pos = this->l_state->read(core::VecCoordId::position());

        for (unsigned i=0;i<d_edges.getValue().size();i++) {
            const Edge & edge = this->d_edges.getValue()[i];

            glVertex3dv(pos[edge[0]].data());
            glVertex3dv(pos[edge[1]].data());
        }
        glEnd();
    }

    void project(unsigned eid, const defaulttype::Vector3 & P, core::topology::BaseMeshTopology::Edge & edge, defaulttype::Vector2 & factor) const {
        edge = d_edges.getValue()[eid];

        const helper::ReadAccessor<Data <VecCoord> >& x = this->getState()->read(core::VecCoordId::position());

        double fact_u;
        double fact_v;

        Coord v = x[edge[1]] - x[edge[0]];
        fact_v = dot (P - x[edge[0]],v) / dot (v,v);

        if (fact_v<0.0) fact_v = 0.0;
        else if (fact_v>1.0) fact_v = 1.0;

        fact_u = 1.0-fact_v;

        factor[0] = fact_u;
        factor[1] = fact_v;
    }
};

}

}
