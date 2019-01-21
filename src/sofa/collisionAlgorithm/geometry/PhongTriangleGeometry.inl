#pragma once

#include <sofa/collisionAlgorithm/geometry/PhongTriangleGeometry.h>
#include <sofa/collisionAlgorithm/proximity/TriangleProximity.h>
#include <sofa/collisionAlgorithm/iterators/DefaultElementIterator.h>

namespace sofa
{

namespace collisionAlgorithm
{

template<class DataTypes>
void PhongTriangleGeometry<DataTypes>::prepareDetection() {
    d_triangles.prepareDetection();
}

template<class DataTypes>
void PhongTriangleGeometry<DataTypes>::init() {
    d_triangles.init();
}

}

}
