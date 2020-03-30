#pragma once

#include <sofa/collisionAlgorithm/BaseProximity.h>

namespace sofa
{

namespace collisionAlgorithm
{


class PointProximity {
public:
    PointProximity(unsigned eid)
    : m_eid(eid){}

    static inline PointProximity create(unsigned eid,CONTROL_POINT c) {
        return PointProximity(eid);
    }

    template<class MatrixDerivRowIterator>
    inline void addContributions(MatrixDerivRowIterator & it, const defaulttype::Vector3 & N) const {
        it.addCol(m_eid, N);
    }

    unsigned getElementId() const {
        return m_eid;
    }

    constexpr static CONTROL_POINT nbControlPoints() {
        return CONTROL_1;
    }

    unsigned m_eid;
};

}

}
