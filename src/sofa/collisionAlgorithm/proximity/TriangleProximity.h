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

    unsigned m_eid;
    unsigned m_p0,m_p1,m_p2;
    double m_f0,m_f1,m_f2;
};

}

}
