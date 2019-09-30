#pragma once

#include <sofa/collisionAlgorithm/BaseProximity.h>

namespace sofa
{

namespace collisionAlgorithm
{

class PointProximity {
public:
    PointProximity(unsigned eid,sofa::defaulttype::Vector3 _normal = sofa::defaulttype::Vector3())
    : m_eid(eid), m_normal(_normal){}

    template<class MatrixDerivRowIterator>
    inline void addContributions(MatrixDerivRowIterator & it, const defaulttype::Vector3 & N) const {
        it.addCol(m_eid, N);
    }

    unsigned getElementId() const {
        return m_eid;
    }

    template<class CONTAINER>
    static BaseProximity::SPtr createProximity(const CONTAINER * container, const PointProximity & data) {
        return BaseProximity::SPtr(new TBaseProximity<CONTAINER, PointProximity>(container,data));
    }

    unsigned m_eid;
    sofa::defaulttype::Vector3 m_normal;
};

}

}
