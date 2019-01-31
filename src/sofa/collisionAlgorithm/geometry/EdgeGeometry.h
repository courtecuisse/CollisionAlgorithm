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
        sofa::core::topology::BaseMeshTopology::Edge edge = m_geo->getEdge(m_eid);
        defaulttype::Vector2 factor;
        m_geo->project(P, edge[0], edge[1], factor);

        return BaseProximity::create<EdgeProximity<GEOMETRY> >(m_geo,m_eid, edge[0],edge[1],factor[0],factor[1]);
    }

    inline BaseProximity::SPtr center() const {
        sofa::core::topology::BaseMeshTopology::Edge edge = m_geo->getEdge(m_eid);

        return BaseProximity::create<EdgeProximity<GEOMETRY> >(m_geo,m_eid, edge[0],edge[1],0.5,0.5);
    }

    inline defaulttype::BoundingBox getBBox() const {
        sofa::core::topology::BaseMeshTopology::Edge edge = m_geo->getEdge(m_eid);

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

    inline const Edge & getEdge(unsigned eid) const {
        return d_edges.getValue()[eid];
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

    void project(const defaulttype::Vector3 & P, unsigned p0, unsigned p1, defaulttype::Vector2 & factor) const {
        const helper::ReadAccessor<Data <VecCoord> >& x = this->getState()->read(core::VecCoordId::position());

        double fact_u;
        double fact_v;

        Coord v = x[p1] - x[p0];
        fact_v = dot (P - x[p0],v) / dot (v,v);

        if (fact_v<0.0) fact_v = 0.0;
        else if (fact_v>1.0) fact_v = 1.0;

        fact_u = 1.0-fact_v;

        factor[0] = fact_u;
        factor[1] = fact_v;
    }
};

}

}
