#pragma once

#include <sofa/collisionAlgorithm/BaseProximity.h>

namespace sofa
{

namespace collisionAlgorithm
{

class TetrahedronProximity {
public:
    typedef sofa::core::topology::Topology::TetrahedronID index_type;

    enum { CONTROL_POINTS = 4};

    TetrahedronProximity(index_type eid,index_type p0,index_type p1,index_type p2, index_type p3, double f0,double f1,double f2,double f3)
    : m_eid(eid), m_p0(p0), m_p1(p1), m_p2(p2), m_p3(p3), m_f0(f0), m_f1(f1), m_f2(f2), m_f3(f3) {}

    static inline TetrahedronProximity create(index_type eid,index_type p0, index_type p1,index_type p2, index_type p3, CONTROL_POINT pid) {
        if (pid == CONTROL_0) return TetrahedronProximity(eid, p0,p1,p2,p3, 1, 0, 0, 0);
        else if (pid == CONTROL_1) return TetrahedronProximity(eid, p0,p1,p2,p3, 0, 1, 0, 0);
        else if (pid == CONTROL_2) return TetrahedronProximity(eid, p0,p1,p2,p3, 0, 0, 1, 0);
        else if (pid == CONTROL_3) return TetrahedronProximity(eid, p0,p1,p2,p3, 0, 0, 0, 1);

        return TetrahedronProximity(eid, p0,p1,p2,p3, 0.25, 0.25, 0.25, 0.25);
    }

    static inline TetrahedronProximity create(index_type eid,const helper::fixed_array<index_type,4> & tetra, CONTROL_POINT pid) {
        return create(eid, tetra[0], tetra[1], tetra[2], tetra[3], pid);
    }

    template<class MatrixDerivRowIterator>
    inline void addContributions(MatrixDerivRowIterator & it, const defaulttype::Vector3 & N) const {
        it.addCol(m_p0, N * m_f0);
        it.addCol(m_p1, N * m_f1);
        it.addCol(m_p2, N * m_f2);
        it.addCol(m_p3, N * m_f3);
    }

    index_type getElementId() const {
        return m_eid;
    }

    constexpr static CONTROL_POINT nbControlPoints() {
        return CONTROL_4;
    }

    index_type m_eid;
    index_type m_p0,m_p1,m_p2,m_p3;
    double m_f0,m_f1,m_f2,m_f3;
};

}

}
