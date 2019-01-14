#pragma once

#include <sofa/collisionAlgorithm/BaseElement.h>
#include <sofa/collisionAlgorithm/geometry/PointGeometry.h>
#include <sofa/collisionAlgorithm/proximity/PointProximity.h>

namespace sofa
{

namespace collisionAlgorithm
{

template<class DataTypes>
class PointElementIterator : public DefaultElement {
public:

    typedef sofa::core::behavior::MechanicalState<DataTypes> State;
    typedef typename DataTypes::Coord Coord;
    typedef typename DataTypes::VecCoord VecCoord;
    typedef Data<VecCoord> DataVecCoord;

    PointElementIterator(const PointGeometry<DataTypes> * geo) : m_geometry(geo) {
        m_state = m_geometry->l_state.get();
    }

    BaseProximity::SPtr project(const defaulttype::Vector3 & /*P*/) const {
        return center();
    }

    bool end(const BaseGeometry */*geo*/) const {
        return id() >= m_state->getSize();
    }

    virtual BaseProximity::SPtr center() const {
        return BaseProximity::SPtr(new PointProximity<DataTypes>(id(),m_geometry->l_state.get()));
    }

    virtual defaulttype::BoundingBox getBBox() const {
        const helper::ReadAccessor<Data <VecCoord> >& x = *m_state->read(core::VecCoordId::position());
        defaulttype::BoundingBox bbox;
        bbox.include(x[id()]);
        return bbox;
    }

    const PointGeometry<DataTypes> * m_geometry;
    State * m_state;
};

}

}
