#pragma once

#include <sofa/collisionAlgorithm/BaseProximity.h>

namespace sofa {

namespace collisionAlgorithm {

class EdgeProximity {
public:
    EdgeProximity(unsigned eid,unsigned p0,unsigned p1,double f0,double f1)
    : m_eid(eid), m_p0(p0), m_p1(p1), m_f0(f0), m_f1(f1) {}

    unsigned m_eid;
    unsigned m_p0,m_p1;
    double m_f0,m_f1;
};


}

}
