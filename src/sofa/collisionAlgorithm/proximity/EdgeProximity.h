#pragma once

#include <sofa/collisionAlgorithm/geometry/EdgeGeometry.h>

namespace sofa {

namespace collisionAlgorithm {

class EdgeProximity : public ConstraintProximity {
public :

    EdgeProximity(const EdgeElement * elmt,double f1,double f2) : ConstraintProximity(elmt) {
        m_fact[0] = f1;
        m_fact[1] = f2;
    }

    inline EdgeElement * element() const {
        return (EdgeElement*) m_element;
    }

    defaulttype::Vector3 getPosition(core::VecCoordId v) const {
        const helper::ReadAccessor<DataVecCoord> pos = m_state->read(v);
        return pos[element()->m_pid[0]] * m_fact[0] + pos[element()->m_pid[1]] * m_fact[1];
    }

    defaulttype::Vector3 getNormal() const {
        return defaulttype::Vector3(1,0,0);
    }

    std::map<unsigned,double> getContributions() const {
        std::map<unsigned,double> res;

        res[element()->m_pid[0]] = m_fact[0];
        res[element()->m_pid[1]] = m_fact[1];

        return res;
    }

protected:
    double m_fact[2];
};

}

}
