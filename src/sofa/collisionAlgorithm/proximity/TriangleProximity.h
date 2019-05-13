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

    template<class MatrixDerivRowIterator>
    inline void addContributions(MatrixDerivRowIterator & it, const defaulttype::Vector3 & N) const {
        it.addCol(m_p0, N * m_f0);
        it.addCol(m_p1, N * m_f1);
        it.addCol(m_p2, N * m_f2);
    }

    template<class CONTAINER>
    static TriangleProximity center(const CONTAINER * container, unsigned eid) {
        const core::topology::BaseMeshTopology::Triangle & triangle = container->getTriangle(eid);

        return TriangleProximity(eid, triangle[0], triangle[1], triangle[2], 0.3333, 0.3333, 0.3333);
    }

    template<class CONTAINER>
    static defaulttype::BoundingBox getBBox(const CONTAINER * container, unsigned eid) {
        const core::topology::BaseMeshTopology::Triangle & triangle = container->getTriangle(eid);
        const helper::ReadAccessor<Data <typename CONTAINER::VecCoord> >& x = container->getState()->read(core::VecCoordId::position());

        defaulttype::BoundingBox bbox;
        bbox.include(x[triangle[0]]);
        bbox.include(x[triangle[1]]);
        bbox.include(x[triangle[2]]);
        return bbox;
    }

    unsigned getElementId() const {
        return m_eid;
    }

    unsigned m_eid;
    unsigned m_p0,m_p1,m_p2;
    double m_f0,m_f1,m_f2;
};

}

}
