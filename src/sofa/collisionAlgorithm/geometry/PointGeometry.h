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

    DataContainer<DataPointContainer<GEOMETRY> > d_points;

    PointGeometry()
    : d_points(initData(&d_points,"points", "Points Container" )) {}

    void init() {
        Inherit::init();

        ///To remove if we think every input has to be explicit
        if(d_points.getValue().empty())
        {
            msg_warning(this) << "Points are not set (data is empty). Will set from state if present in the same context";
            d_points.setParent(this->l_state->findData("position"));
        }
    }

};

}

}
