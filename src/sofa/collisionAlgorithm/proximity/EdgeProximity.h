#pragma once

#include <sofa/collisionAlgorithm/BaseProximity.h>

namespace sofa {

namespace collisionAlgorithm {

class EdgeProximity {
public:
    EdgeProximity(unsigned eid,unsigned p0,unsigned p1,double f0,double f1)
    : m_eid(eid), m_p0(p0), m_p1(p1), m_f0(f0), m_f1(f1) {}

    template<class MatrixDerivRowIterator>
    inline void addContributions(MatrixDerivRowIterator & it, const defaulttype::Vector3 & N) const {
        it.addCol(m_p0, N * m_f0);
        it.addCol(m_p1, N * m_f1);
    }

    template<class CONTAINER>
    static EdgeProximity center(const CONTAINER * container, unsigned eid) {
        const core::topology::BaseMeshTopology::Edge & edge = container->getEdge(eid);

        return EdgeProximity(eid, edge[0], edge[1], 0.5, 0.5);
    }

    template<class CONTAINER>
    static defaulttype::BoundingBox getBBox(const CONTAINER * container, unsigned eid) {
        const core::topology::BaseMeshTopology::Edge & edge = container->getEdge(eid);
        const helper::ReadAccessor<Data <typename CONTAINER::VecCoord> >& x = container->getState()->read(core::VecCoordId::position());

        defaulttype::BoundingBox bbox;
        bbox.include(x[edge[0]]);
        bbox.include(x[edge[1]]);
        return bbox;
    }

    unsigned getElementId() const {
        return m_eid;
    }

    unsigned m_eid;
    unsigned m_p0,m_p1;
    double m_f0,m_f1;
};


}

}
