#pragma once

#include <sofa/collisionAlgorithm/BaseProximity.h>

namespace sofa
{

namespace collisionAlgorithm
{

class TriangleProximity {
public:

    TriangleProximity(unsigned eid,unsigned p0,unsigned p1,unsigned p2, double f0,double f1,double f2)
    : m_eid(eid), m_p0(p0), m_p1(p1), m_p2(p2), m_f0(f0), m_f1(f1), m_f2(f2) {}

    static inline TriangleProximity create(unsigned eid,unsigned p0,unsigned p1,unsigned p2, CONTROL_POINT pid) {
        if (pid == 0) return TriangleProximity(eid, p0,p1,p2, 1, 0, 0);
        else if (pid == 1) return TriangleProximity(eid, p0,p1,p2, 0, 1, 0);
        else if (pid == 2) return TriangleProximity(eid, p0,p1,p2, 0, 0, 1);

        return TriangleProximity(eid, p0,p1,p2, 1.0/3.0, 1.0/3.0, 1.0/3.0);
    }

    static inline TriangleProximity create(unsigned eid,const helper::fixed_array<unsigned,3> & tri, CONTROL_POINT pid) {
        return create(eid,tri[0],tri[1],tri[2],pid);
    }

    template<class MatrixDerivRowIterator>
    inline void addContributions(MatrixDerivRowIterator & it, const defaulttype::Vector3 & N) const {
        it.addCol(m_p0, N * m_f0);
        it.addCol(m_p1, N * m_f1);
        it.addCol(m_p2, N * m_f2);
    }

    unsigned getElementId() const {
        return m_eid;
    }

    constexpr static CONTROL_POINT nbControlPoints() {
        return CONTROL_3;
    }

    void getConstraintMatrix(int cId, sofa::defaulttype::BaseMatrix * J_from, double fact){
        J_from->add(cId, m_p0, m_f0*fact);
        J_from->add(cId, m_p1, m_f1*fact);
        J_from->add(cId, m_p2, m_f2*fact);
    }

    unsigned m_eid;
    unsigned m_p0,m_p1,m_p2;
    double m_f0,m_f1,m_f2;
};

}

}
