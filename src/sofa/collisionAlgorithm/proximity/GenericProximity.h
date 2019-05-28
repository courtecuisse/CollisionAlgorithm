#pragma once

#include <sofa/collisionAlgorithm/BaseProximity.h>

namespace sofa {

namespace collisionAlgorithm {


class GenericProximity {
public:
    GenericProximity(unsigned eid, helper::vector<std::pair<unsigned,double> > prox)
    : m_eid(eid), m_prox(prox) {}

    template<class MatrixDerivRowIterator>
    inline void addContributions(MatrixDerivRowIterator & it, const defaulttype::Vector3 & N) const {
        for (unsigned i=0;i<m_prox.size();i++) {
            it.addCol(m_prox[i].first, N * m_prox[i].second);
        }
    }

    unsigned getElementId() const {
        return m_eid;
    }

    unsigned m_eid;
    helper::vector<std::pair<unsigned, double> > m_prox;
};

}

}

