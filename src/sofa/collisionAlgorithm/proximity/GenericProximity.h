#pragma once

#include <sofa/collisionAlgorithm/BaseProximity.h>

namespace sofa {

namespace collisionAlgorithm {

template<CONTROL_POINT SIZE>
class GenericProximity {
public:
    typedef BaseProximity::index_type index_type;

    GenericProximity(index_type eid, helper::vector<std::pair<index_type,double> > prox)
    : m_eid(eid), m_prox(prox) {}

    static inline GenericProximity create(index_type eid, CONTROL_POINT c) {
        helper::vector<std::pair<index_type,double> > prox;

//        helper::vector<std::pair<index_type,double> > prox;
//        for (index_type i=0;i<Element::size();i++) {
//            prox.push_back(std::pair<index_type,double>(eid, 1.0/Element::size()));
//        }
//        if (c == CONTROL_1) prox

        return GenericProximity(eid,prox);
    }

    template<class MatrixDerivRowIterator>
    inline void addContributions(MatrixDerivRowIterator & it, const defaulttype::Vector3 & N) const {
        for (index_type i=0;i<m_prox.size();i++) {
            it.addCol(m_prox[i].first, N * m_prox[i].second);
        }
    }

    index_type getElementId() const {
        return m_eid;
    }

    constexpr static CONTROL_POINT nbControlPoints() {
        return SIZE;
    }

    index_type m_eid;
    helper::vector<std::pair<index_type, double> > m_prox;
};

}

}

