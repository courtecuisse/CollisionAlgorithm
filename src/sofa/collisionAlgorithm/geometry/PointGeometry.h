#pragma once

#include <sofa/collisionAlgorithm/BaseGeometry.h>
#include <sofa/collisionAlgorithm/iterators/DefaultElementIterator.h>
#include <sofa/collisionAlgorithm/proximity/PointProximity.h>
#include <sofa/collisionAlgorithm/elements/DataPointElement.h>

namespace sofa {

namespace collisionAlgorithm {

template<class DataTypes>
class PointGeometry : public BaseGeometry<DataTypes> {
public:
    typedef DataTypes TDataTypes;
    typedef BaseGeometry<DataTypes> Inherit;
    typedef PointGeometry<DataTypes> GEOMETRY;

    SOFA_CLASS(GEOMETRY,Inherit);

    DataPointContainer<GEOMETRY> d_points = { this };

    PointGeometry()
    : d_points(initData(&d_points, "points", "Points container")) {}

};

}

}
