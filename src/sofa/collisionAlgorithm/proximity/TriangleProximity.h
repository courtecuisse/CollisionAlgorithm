//#pragma once

//#include <sofa/collisionAlgorithm/BaseProximity.h>

//namespace sofa
//{

//namespace collisionAlgorithm
//{

//class TriangleProximity {
//public:
//    typedef sofa::core::topology::Topology::TriangleID Index;

//    TriangleProximity(Index eid,Index p0,Index p1,Index p2, double f0,double f1,double f2)
//    : m_eid(eid), m_p0(p0), m_p1(p1), m_p2(p2), m_f0(f0), m_f1(f1), m_f2(f2) {}

//    static inline TriangleProximity create(Index eid,Index p0,Index p1,Index p2, CONTROL_POINT pid) {
//        if (pid == 0) return TriangleProximity(eid, p0,p1,p2, 1, 0, 0);
//        else if (pid == 1) return TriangleProximity(eid, p0,p1,p2, 0, 1, 0);
//        else if (pid == 2) return TriangleProximity(eid, p0,p1,p2, 0, 0, 1);

//        return TriangleProximity(eid, p0,p1,p2, 1.0/3.0, 1.0/3.0, 1.0/3.0);
//    }

//    static inline TriangleProximity create(Index eid,const type::fixed_array<Index,3> & tri, CONTROL_POINT pid) {
//        return create(eid,tri[0],tri[1],tri[2],pid);
//    }

//    template<class MatrixDerivRowIterator>
//    inline void addContributions(MatrixDerivRowIterator & it, const sofa::type::Vector3 & N) const {
//        it.addCol(m_p0, N * m_f0);
//        it.addCol(m_p1, N * m_f1);
//        it.addCol(m_p2, N * m_f2);
//    }

//    Index getElementId() const {
//        return m_eid;
//    }

//    constexpr static CONTROL_POINT nbControlPoints() {
//        return CONTROL_3;
//    }

//    Index m_eid;
//    Index m_p0,m_p1,m_p2;
//    double m_f0,m_f1,m_f2;
//};

//}

//}
