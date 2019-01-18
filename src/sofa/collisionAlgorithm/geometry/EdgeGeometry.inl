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
    return DefaultElementIterator<GEOMETRY, EdgeProximity<GEOMETRY> >::create(this, d_edges.getValue(), eid);
}

template<class DataTypes>
void EdgeGeometry<DataTypes>::project(unsigned eid, const defaulttype::Vector3 & P, core::topology::BaseMeshTopology::Edge & edge, defaulttype::Vector2 & factor) const {
    edge = d_edges.getValue()[eid];

    const helper::ReadAccessor<Data <VecCoord> >& x = *this->l_state->read(core::VecCoordId::position());

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

}

}
