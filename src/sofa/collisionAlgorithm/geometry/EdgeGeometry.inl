#pragma once

#include <sofa/collisionAlgorithm/geometry/EdgeGeometry.h>
#include <sofa/collisionAlgorithm/element/EdgeElement.h>

namespace sofa
{

namespace collisionAlgorithm
{

template<class DataTypes>
BaseElement::Iterator EdgeGeometry<DataTypes>::begin(unsigned eid) const {
    return DefaultElement::Iterator(eid,new EdgeElementIterator<DataTypes>(this));
}

}

}
