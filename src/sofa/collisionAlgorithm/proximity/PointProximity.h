#pragma once

#include <sofa/collisionAlgorithm/BaseProximity.h>

namespace sofa
{

namespace collisionAlgorithm
{

class PointProximity {
public:
    PointProximity(unsigned eid)
    : m_eid(eid) {}

    template<class MatrixDerivRowIterator>
    inline void addContributions(MatrixDerivRowIterator & it, const defaulttype::Vector3 & N) const {
        it.addCol(m_eid, N);
    }

    template<class CONTAINER>
    static PointProximity center(const CONTAINER * /*container*/, unsigned eid) {
        return PointProximity(eid);
    }

    template<class CONTAINER>
    static defaulttype::BoundingBox getBBox(const CONTAINER * container, unsigned eid) {
        const helper::ReadAccessor<Data <typename CONTAINER::VecCoord> >& x = container->getState()->read(core::VecCoordId::position());

        defaulttype::BoundingBox bbox;
        bbox.include(x[eid]);
        return bbox;
    }

    unsigned getElementId() const {
        return m_eid;
    }

    unsigned m_eid;
};

}

}
