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

    PhongTriangleProximity(State * state,unsigned p1,unsigned p2,unsigned p3,double f1,double f2,double f3, const defaulttype::Vector3 & N0, const defaulttype::Vector3 & N1, const defaulttype::Vector3 & N2)
    : TriangleProximity<DataTypes>(state,p1,p2,p3,f1,f2,f3,N0)
    , m_normal1(N1)
    , m_normal2(N2) {}

    inline defaulttype::Vector3 getNormal() const {
        return this->m_normal * this->m_fact[0] +
               m_normal1 * this->m_fact[1] +
               m_normal2 * this->m_fact[2];
    }

protected:
    const defaulttype::Vector3 & m_normal1;
    const defaulttype::Vector3 & m_normal2;

};

}

}
