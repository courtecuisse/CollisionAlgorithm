﻿#pragma once

#include <sofa/collisionAlgorithm/geometry/PointGeometry.h>

namespace sofa {

namespace collisionAlgorithm {

class PointProximity : public ConstraintProximity {
public :
    PointProximity(const PointElement * elmt) : ConstraintProximity(elmt) {}

    defaulttype::Vector3 getPosition(core::VecCoordId v) const {
        const helper::ReadAccessor<DataVecCoord> & pos = m_state->read(v);
        return pos[element()->m_pid];
    }

    defaulttype::Vector3 getNormal() const {
        return defaulttype::Vector3(1,0,0);
    }

    std::map<unsigned,double> getContributions() const {
        std::map<unsigned,double> res;

        res[element()->m_pid] = 1.0;

        return res;
    }

    inline PointElement * element() const {
        return (PointElement*) m_element;
    }
};

}

}