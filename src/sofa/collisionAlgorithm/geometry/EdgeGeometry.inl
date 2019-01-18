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
    return DefaultElementIterator<EdgeGeometry<DataTypes>, EdgeProximity<DataTypes> >::create(d_edges.getValue(),this, eid);
}

}

}
