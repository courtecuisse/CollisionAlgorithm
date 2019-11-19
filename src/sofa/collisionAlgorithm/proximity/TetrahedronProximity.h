#pragma once

#include <sofa/collisionAlgorithm/BaseProximity.h>

namespace sofa
{

namespace collisionAlgorithm
{

class TetrahedronProximity {
public:
    TetrahedronProximity(unsigned eid,unsigned p0,unsigned p1,unsigned p2, unsigned p3, double f0,double f1,double f2,double f3)
    : m_eid(eid), m_p0(p0), m_p1(p1), m_p2(p2), m_p3(p3), m_f0(f0), m_f1(f1), m_f2(f2), m_f3(f3) {}

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

    unsigned m_eid;
    unsigned m_p0,m_p1,m_p2,m_p3;
    double m_f0,m_f1,m_f2,m_f3;
};

}

}
