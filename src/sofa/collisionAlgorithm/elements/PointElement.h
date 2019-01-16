#pragma once

#include <sofa/collisionAlgorithm/BaseElement.h>
#include <sofa/collisionAlgorithm/geometry/PointGeometry.h>
#include <sofa/collisionAlgorithm/proximity/PointProximity.h>

namespace sofa
{

namespace collisionAlgorithm
{

template<class DataTypes>
class PointElement : public DefaultElement {
public:

    typedef sofa::core::behavior::MechanicalState<DataTypes> State;
    typedef typename DataTypes::Coord Coord;
    typedef typename DataTypes::VecCoord VecCoord;
    typedef Data<VecCoord> DataVecCoord;

    PointElement(State * state) : m_state(state) {}

    BaseProximity::SPtr project(const defaulttype::Vector3 & /*P*/) const {
        return center();
    }

    virtual BaseProximity::SPtr center() const {
        return BaseProximity::SPtr(new PointProximity<DataTypes>(id(),m_state));
    }

    virtual defaulttype::BoundingBox getBBox() const {
        const helper::ReadAccessor<Data <VecCoord> >& x = *m_state->read(core::VecCoordId::position());
        defaulttype::BoundingBox bbox;
        bbox.include(x[id()]);
        return bbox;
    }

    State * m_state;
};

}

}
