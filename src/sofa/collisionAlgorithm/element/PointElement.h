#pragma once

#include <sofa/collisionAlgorithm/BaseElement.h>
#include <sofa/collisionAlgorithm/geometry/PointGeometry.h>
#include <sofa/collisionAlgorithm/proximity/PointProximity.h>

namespace sofa
{

namespace collisionAlgorithm
{

template<class DataTypes>
class PointElementIterator : public ElementIterator {
public:

    typedef sofa::core::behavior::MechanicalState<DataTypes> State;

    PointElementIterator(const PointGeometry<DataTypes> * geo) : m_geometry(geo) {
        m_state = m_geometry->l_state.get();
    }

    BaseProximity::SPtr project(const defaulttype::Vector3 & /*P*/) const {
        return center();
    }

    virtual BaseProximity::SPtr center() const {
        return BaseProximity::SPtr(new PointProximity<DataTypes>(id(),m_geometry->l_state.get()));
    }

    const PointGeometry<DataTypes> * m_geometry;
    State * m_state;
};

}

}
