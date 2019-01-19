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

    SOFA_CLASS(GEOMETRY,Inherit);

    virtual BaseElementIterator::UPtr getElementIterator(unsigned eid = 0) const;

    virtual void draw(const core::visual::VisualParams *vparams) override;

    inline defaulttype::Vector3 getNormal(const PointProximity<GEOMETRY> * /*prox*/) const {
        return defaulttype::Vector3(1,0,0);
    }

    //default implementation
    template<class DERIVED_GEOMETRY>
    inline Coord getPosition(const PointProximity<DERIVED_GEOMETRY> * prox, core::VecCoordId v = core::VecCoordId::position()) const {
        return this->l_state->read(v)->getValue()[prox->m_pid];
    }
};

}

}
