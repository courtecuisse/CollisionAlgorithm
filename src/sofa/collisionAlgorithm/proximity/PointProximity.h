#pragma once

#include <sofa/collisionAlgorithm/BaseProximity.h>

namespace sofa
{

namespace collisionAlgorithm
{


class PointProximity {
public:
    typedef sofa::core::topology::Topology::PointID index_type;

    PointProximity(index_type eid)
    : m_eid(eid){}

    static inline PointProximity create(index_type eid,CONTROL_POINT c) {
        return PointProximity(eid);
    }

    template<class MatrixDerivRowIterator>
    inline void addContributions(MatrixDerivRowIterator & it, const defaulttype::Vector3 & N) const {
        it.addCol(m_eid, N);
    }

    index_type getElementId() const {
        return m_eid;
    }

    constexpr static CONTROL_POINT nbControlPoints() {
        return CONTROL_1;
    }

    index_type m_eid;
};

}

}
