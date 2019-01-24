#pragma once

#include <sofa/collisionAlgorithm/geometry/PointGeometry.h>
#include <sofa/collisionAlgorithm/proximity/EdgeProximity.h>

namespace sofa {

namespace collisionAlgorithm {

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
        return DefaultElementIterator<GEOMETRY >::create(this, d_edges.getValue().size(), eid);
    }

    inline BaseProximity::SPtr project(unsigned eid, const defaulttype::Vector3 & P) const {
        core::topology::BaseMeshTopology::Edge edge;
        defaulttype::Vector2 factor;
        project(eid, P, edge, factor);

        return BaseProximity::create<EdgeProximity<GEOMETRY> >(this,edge[0],edge[1],factor[0],factor[1]);
    }

    inline BaseProximity::SPtr center(unsigned eid) const {
        const core::topology::BaseMeshTopology::Edge & edge = d_edges.getValue()[eid];
        return BaseProximity::create<EdgeProximity<GEOMETRY> >(this,edge[0],edge[1],0.5,0.5);
    }

    inline defaulttype::BoundingBox getBBox(unsigned eid) const {
        const core::topology::BaseMeshTopology::Edge & edge = d_edges.getValue()[eid];
        const helper::ReadAccessor<Data <VecCoord> >& x = this->getState()->read(core::VecCoordId::position());
        defaulttype::BoundingBox bbox;
        bbox.include(x[edge[0]]);
        bbox.include(x[edge[1]]);
        return bbox;
    }

    void draw(const core::visual::VisualParams * vparams) {
        if (! vparams->displayFlags().getShowCollisionModels())
            return;

        if (this->d_color.getValue()[3] == 0.0)
            return;

        glDisable(GL_LIGHTING);


    }

protected:

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
