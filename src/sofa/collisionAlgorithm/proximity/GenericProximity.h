#pragma once

#include <sofa/collisionAlgorithm/BaseProximity.h>

namespace sofa {

namespace collisionAlgorithm {

template<CONTROL_POINT SIZE>
class GenericProximity {
public:

    GenericProximity(unsigned eid, helper::vector<std::pair<unsigned,double> > prox)
    : m_eid(eid), m_prox(prox) {}

    static inline GenericProximity create(unsigned eid, CONTROL_POINT c) {
        helper::vector<std::pair<unsigned,double> > prox;

//        helper::vector<std::pair<unsigned,double> > prox;
//        for (unsigned i=0;i<Element::size();i++) {
//            prox.push_back(std::pair<unsigned,double>(eid, 1.0/Element::size()));
//        }
//        if (c == CONTROL_1) prox

        return GenericProximity(eid,prox);
    }

    template<class MatrixDerivRowIterator>
    inline void addContributions(MatrixDerivRowIterator & it, const defaulttype::Vector3 & N) const {
        for (unsigned i=0;i<m_prox.size();i++) {
            it.addCol(m_prox[i].first, N * m_prox[i].second);
        }
    }

    unsigned getElementId() const {
        return m_eid;
    }

    constexpr static CONTROL_POINT nbControlPoints() {
        return SIZE;
    }

    unsigned m_eid;
    helper::vector<std::pair<unsigned, double> > m_prox;
};

}

}

