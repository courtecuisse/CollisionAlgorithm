﻿#pragma once

#include <sofa/collisionAlgorithm/BaseProximity.h>

namespace sofa
{

namespace collisionAlgorithm
{

class TetrahedronProximity {
public:
    enum { CONTROL_POINTS = 4};

    TetrahedronProximity(unsigned eid,unsigned p0,unsigned p1,unsigned p2, unsigned p3, double f0,double f1,double f2,double f3)
    : m_eid(eid), m_p0(p0), m_p1(p1), m_p2(p2), m_p3(p3), m_f0(f0), m_f1(f1), m_f2(f2), m_f3(f3) {}

    static inline TetrahedronProximity create(unsigned eid,unsigned p0, unsigned p1,unsigned p2, unsigned p3, CONTROL_POINT pid) {
        if (pid == CONTROL_0) return TetrahedronProximity(eid, p0,p1,p2,p3, 1, 0, 0, 0);
        else if (pid == CONTROL_1) return TetrahedronProximity(eid, p0,p1,p2,p3, 0, 1, 0, 0);
        else if (pid == CONTROL_2) return TetrahedronProximity(eid, p0,p1,p2,p3, 0, 0, 1, 0);
        else if (pid == CONTROL_3) return TetrahedronProximity(eid, p0,p1,p2,p3, 0, 0, 0, 1);

        return TetrahedronProximity(eid, p0,p1,p2,p3, 0.25, 0.25, 0.25, 0.25);
    }

    static inline TetrahedronProximity create(unsigned eid,const helper::fixed_array<unsigned,4> & tetra, CONTROL_POINT pid) {
        return create(eid, tetra[0], tetra[1], tetra[2], tetra[3], pid);
    }

    template<class MatrixDerivRowIterator>
    inline void addContributions(MatrixDerivRowIterator & it, const defaulttype::Vector3 & N) const {
        it.addCol(m_p0, N * m_f0);
        it.addCol(m_p1, N * m_f1);
        it.addCol(m_p2, N * m_f2);
        it.addCol(m_p3, N * m_f3);
    }

    unsigned getElementId() const {
        return m_eid;
    }

    constexpr static CONTROL_POINT nbControlPoints() {
        return CONTROL_4;
    }

    inline void addColInJ0(int cId, sofa::defaulttype::BaseMatrix * J0, double fact) const{
        J0->add(cId, m_p0, m_f0*fact);
        J0->add(cId, m_p1, m_f1*fact);
        J0->add(cId, m_p2, m_f2*fact);
        J0->add(cId, m_p3, m_f3*fact);
    }

    unsigned m_eid;
    unsigned m_p0,m_p1,m_p2,m_p3;
    double m_f0,m_f1,m_f2,m_f3;
};

}

}
