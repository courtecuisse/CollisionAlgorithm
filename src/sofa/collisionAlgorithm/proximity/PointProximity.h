//#pragma once

//#include <sofa/collisionAlgorithm/BaseProximity.h>

//namespace sofa
//{

//namespace collisionAlgorithm
//{


//class PointProximity {
//public:
//    typedef sofa::core::topology::Topology::PointID Index;

//    PointProximity(Index eid)
//    : m_eid(eid){}

//    static inline PointProximity create(Index eid,CONTROL_POINT /*c*/) {
//        return PointProximity(eid);
//    }

//    template<class MatrixDerivRowIterator>
//    inline void addContributions(MatrixDerivRowIterator & it, const sofa::type::Vector3 & N) const {
//        it.addCol(m_eid, N);
//    }

//    Index getElementId() const {
//        return m_eid;
//    }

//    constexpr static CONTROL_POINT nbControlPoints() {
//        return CONTROL_1;
//    }

//    Index m_eid;
//};

//}

//}
