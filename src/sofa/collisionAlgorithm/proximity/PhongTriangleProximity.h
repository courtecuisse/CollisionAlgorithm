#pragma once

#include <sofa/collisionAlgorithm/BaseProximity.h>
#include <sofa/collisionAlgorithm/proximity/TriangleProximity.h>

namespace sofa
{

namespace collisionAlgorithm
{

template<class DataTypes>
class PhongTriangleProximity : public TriangleProximity<DataTypes> {
public :

    typedef sofa::core::behavior::MechanicalState<DataTypes> State;

    PhongTriangleProximity(State * state,unsigned tid, unsigned p1,unsigned p2,unsigned p3,double f1,double f2,double f3, const helper::vector<defaulttype::Vector3> & N)
    : TriangleProximity<DataTypes>(state,tid, p1,p2,p3,f1,f2,f3,N) {}

    inline defaulttype::Vector3 getNormal() const {
        return this->m_normalVector[this->m_pid[0]] * this->m_fact[0] +
               this->m_normalVector[this->m_pid[1]] * this->m_fact[1] +
               this->m_normalVector[this->m_pid[2]] * this->m_fact[2];
    }

};

}

}
