#pragma once

#include <sofa/collisionAlgorithm/BaseGeometry.h>

namespace sofa
{

namespace collisionAlgorithm
{

template<class DataTypes>
class PointGeometry : public TBaseGeometry<DataTypes> {
public:
    typedef TBaseGeometry<DataTypes> Inherit;
    typedef DataTypes TDataTypes;
    typedef typename DataTypes::Coord Coord;
    typedef typename DataTypes::VecCoord VecCoord;
    typedef Data<VecCoord> DataVecCoord;

    SOFA_CLASS(SOFA_TEMPLATE(PointGeometry,DataTypes),Inherit);

    virtual BaseElementIterator::UPtr begin(unsigned eid = 0) const;

    virtual void draw(const core::visual::VisualParams *vparams) override;

};

}

}
