#pragma once

#include <sofa/collisionAlgorithm/geometry/EdgeGeometry.h>
#include <sofa/collisionAlgorithm/proximity/EdgeProximity.h>
#include <sofa/collisionAlgorithm/iterators/DefaultElementIterator.h>

namespace sofa
{

namespace collisionAlgorithm
{

template<class DataTypes>
BaseElementIterator::UPtr EdgeGeometry<DataTypes>::begin(unsigned eid) const {
    return BaseElementIterator::UPtr(new DefaultElementIterator(eid,d_edges.getValue().size(),this));
}

template<class DataTypes>
BaseProximity::SPtr EdgeGeometry<DataTypes>::project(unsigned eid, const defaulttype::Vector3 & P) const {
    const core::topology::BaseMeshTopology::Edge edge = d_edges.getValue()[eid];

    const helper::ReadAccessor<Data <VecCoord> >& x = *this->l_state->read(core::VecCoordId::position());

    double fact_u;
    double fact_v;

    Coord v = x[edge[1]] - x[edge[0]];
    fact_v = dot (P - x[edge[0]],v) / dot (v,v);

    if (fact_v<0.0) fact_v = 0.0;
    else if (fact_v>1.0) fact_v = 1.0;

    fact_u = 1.0-fact_v;

    return BaseProximity::SPtr(new EdgeProximity<DataTypes>(edge[0],edge[1],fact_u,fact_v,this->l_state.get()));
}

template<class DataTypes>
BaseProximity::SPtr EdgeGeometry<DataTypes>::center(unsigned eid) const {
    const core::topology::BaseMeshTopology::Edge edge = d_edges.getValue()[eid];
    return BaseProximity::SPtr(new EdgeProximity<DataTypes>(edge[0],edge[1],0.5,0.5,this->l_state.get()));
}

template<class DataTypes>
defaulttype::BoundingBox EdgeGeometry<DataTypes>::getBBox(unsigned eid) const {
    const core::topology::BaseMeshTopology::Edge edge = d_edges.getValue()[eid];
    const helper::ReadAccessor<DataVecCoord >& x = *this->l_state->read(core::VecCoordId::position());
    defaulttype::BoundingBox bbox;
    bbox.include(x[edge[0]]);
    bbox.include(x[edge[1]]);
    return bbox;
}

}

}
