#pragma once

#include <sofa/collisionAlgorithm/BaseProximity.h>

namespace sofa {

namespace collisionAlgorithm {

template<CONTROL_POINT SIZE>
class GenericProximity {
public:
    typedef BaseProximity::Index Index;

    GenericProximity(Index eid, helper::vector<std::pair<Index,double> > prox)
    : m_eid(eid), m_prox(prox) {}

    static inline GenericProximity create(Index eid, CONTROL_POINT c) {
        helper::vector<std::pair<Index,double> > prox;

//        helper::vector<std::pair<Index,double> > prox;
//        for (Index i=0;i<Element::size();i++) {
//            prox.push_back(std::pair<Index,double>(eid, 1.0/Element::size()));
//        }
//        if (c == CONTROL_1) prox

        return GenericProximity(eid,prox);
    }

    template<class MatrixDerivRowIterator>
    inline void addContributions(MatrixDerivRowIterator & it, const defaulttype::Vector3 & N) const {
        for (Index i=0;i<m_prox.size();i++) {
            it.addCol(m_prox[i].first, N * m_prox[i].second);
        }
    }

    Index getElementId() const {
        return m_eid;
    }

    constexpr static CONTROL_POINT nbControlPoints() {
        return SIZE;
    }

    Index m_eid;
    helper::vector<std::pair<Index, double> > m_prox;
};

}

}

