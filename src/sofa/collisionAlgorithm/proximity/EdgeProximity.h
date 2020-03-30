#pragma once

#include <sofa/collisionAlgorithm/BaseProximity.h>

namespace sofa {

namespace collisionAlgorithm {

class EdgeProximity {
public:
    EdgeProximity(unsigned eid,unsigned p0,unsigned p1,double f0,double f1)
    : m_eid(eid), m_p0(p0), m_p1(p1), m_f0(f0), m_f1(f1) {}

    static inline EdgeProximity create(unsigned eid,unsigned p0,unsigned p1,CONTROL_POINT c) {
        if (c == CONTROL_0) return EdgeProximity(eid,p0,p1,1,0);
        else if (c == CONTROL_1) return EdgeProximity(eid,p0,p1,0,1);

        return EdgeProximity(eid,p0,p1,0.5,0.5);
    }

    static inline EdgeProximity create(unsigned eid,const helper::fixed_array<unsigned,2> & edge,CONTROL_POINT c) {
        return create(eid,edge[0],edge[1],c);
    }

    template<class MatrixDerivRowIterator>
    inline void addContributions(MatrixDerivRowIterator & it, const defaulttype::Vector3 & N) const {
        it.addCol(m_p0, N * m_f0);
        it.addCol(m_p1, N * m_f1);
    }

    unsigned getElementId() const {
        return m_eid;
    }

    constexpr static CONTROL_POINT nbControlPoints() {
        return CONTROL_2;
    }

    unsigned m_eid;
    unsigned m_p0,m_p1;
    double m_f0,m_f1;
};


}

}
