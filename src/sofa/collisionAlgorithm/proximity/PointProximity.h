#pragma once

#include <sofa/collisionAlgorithm/BaseProximity.h>

namespace sofa
{

namespace collisionAlgorithm
{

class PointProximity {
public:
    PointProximity(unsigned eid)
    : m_eid(eid) {}

    unsigned m_eid;
};

}

}
