#pragma once

#include <geometry/TriangleGeometry.h>

namespace sofa {

namespace collisionAlgorithm {

class TriangleProximity : public ConstraintProximity {
public :
    TriangleProximity(const TriangleElement * elmt,double f1,double f2,double f3) : ConstraintProximity (elmt) {
        m_fact[0] = f1;
        m_fact[1] = f2;
        m_fact[2] = f3;
    }

    defaulttype::Vector3 getPosition(core::VecCoordId v) const {
        const helper::ReadAccessor<DataVecCoord> & pos = m_state->read(v);

        return pos[element()->m_pid[0]] * m_fact[0] +
               pos[element()->m_pid[1]] * m_fact[1] +
               pos[element()->m_pid[2]] * m_fact[2];
    }

    defaulttype::Vector3 getNormal() const {
        const std::vector<defaulttype::Vector3> & pos = element()->geometry()->m_pointNormal;
        return pos[element()->m_pid[0]] * m_fact[0] +
               pos[element()->m_pid[1]] * m_fact[1] +
               pos[element()->m_pid[2]] * m_fact[2];
    }

    std::map<unsigned,double> getContributions() const {
        std::map<unsigned,double> res;

        res[element()->m_pid[0]] = m_fact[0];
        res[element()->m_pid[1]] = m_fact[1];
        res[element()->m_pid[2]] = m_fact[2];

        return res;
    }

    inline TriangleElement * element() const {
        return (TriangleElement*) m_element;
    }

    double m_fact[3];
};

}

}
