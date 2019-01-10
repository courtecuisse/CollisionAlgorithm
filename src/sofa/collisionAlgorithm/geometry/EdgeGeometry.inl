#pragma once

#include <sofa/collisionAlgorithm/geometry/EdgeGeometry.h>
#include <sofa/collisionAlgorithm/element/EdgeElement.h>

namespace sofa
{

namespace collisionAlgorithm
{

template<class DataTypes>
ElementIterator::UPtr EdgeGeometry<DataTypes>::begin() const {
    return ElementIterator::UPtr(new EdgeElementIterator<DataTypes>(this));
}

template<class DataTypes>
ElementIterator::End EdgeGeometry<DataTypes>::end() const {
    return d_edges.getValue().size();
}

}

}
