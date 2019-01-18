#pragma once

#include <sofa/collisionAlgorithm/BaseGeometry.h>

namespace sofa
{

namespace collisionAlgorithm
{

template<class GEOMETRY>
class PointProximity;

template<class DataTypes>
class PointGeometry : public TBaseGeometry<DataTypes> {
public:
    typedef TBaseGeometry<DataTypes> Inherit;
    typedef PointGeometry<DataTypes> GEOMETRY;
    typedef DataTypes TDataTypes;
    typedef typename DataTypes::Coord Coord;
    typedef typename DataTypes::Deriv Deriv;
    typedef typename DataTypes::VecCoord VecCoord;
    typedef Data<VecCoord> DataVecCoord;

    friend class PointProximity<GEOMETRY>;

    SOFA_CLASS(GEOMETRY,Inherit);

    virtual BaseElementIterator::UPtr begin(unsigned eid = 0) const;

    virtual void draw(const core::visual::VisualParams *vparams) override;

protected:
    inline defaulttype::Vector3 getNormal(unsigned /*pid*/) const {
        return defaulttype::Vector3(1,0,0);
    }

    inline Coord getPosition(core::VecCoordId v, unsigned pid) const {
        return this->l_state->read(v)->getValue()[pid];
    }
};

}

}
