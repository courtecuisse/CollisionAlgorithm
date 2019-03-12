#pragma once

#include <sofa/collisionAlgorithm/BaseProximity.h>

namespace sofa
{

namespace collisionAlgorithm
{

template<class DataTypes>
class TriangleProximity : public TBaseProximity<DataTypes> {
public :

    typedef typename DataTypes::VecCoord VecCoord;
    typedef typename DataTypes::Coord Coord;
    typedef typename DataTypes::Real Real;
    typedef typename DataTypes::VecDeriv VecDeriv;
    typedef typename DataTypes::MatrixDeriv MatrixDeriv;
    typedef typename MatrixDeriv::RowIterator MatrixDerivRowIterator;
    typedef core::objectmodel::Data< VecCoord >        DataVecCoord;
    typedef core::objectmodel::Data< VecDeriv >        DataVecDeriv;
    typedef core::objectmodel::Data< MatrixDeriv >     DataMatrixDeriv;
    typedef sofa::core::behavior::MechanicalState<DataTypes> State;

    TriangleProximity(sofa::core::behavior::MechanicalState<DataTypes> * state,unsigned p0,unsigned p1,unsigned p2,double f0,double f1,double f2,const defaulttype::Vector3 & n)
    : TBaseProximity<DataTypes>(state)
    , m_p0(p0) , m_p1(p1) , m_p2(p2)
    , m_f0(f0) , m_f1(f1) , m_f2(f2)
    , m_normal(n) {}

    inline defaulttype::Vector3 getPosition(core::VecCoordId v = core::VecCoordId::position()) const {
        const helper::ReadAccessor<DataVecCoord> & pos = this->m_state->read(v);

        return pos[m_p0] * m_f0 +
               pos[m_p1] * m_f1 +
               pos[m_p2] * m_f2;
    }

    inline defaulttype::Vector3 getNormal() const {
        return m_normal;
    }

    void addContributions(MatrixDerivRowIterator & it, const defaulttype::Vector3 & N) const {
        it.addCol(m_p0, N * m_f0);
        it.addCol(m_p1, N * m_f1);
        it.addCol(m_p2, N * m_f2);
    }

protected:
    unsigned m_p0,m_p1,m_p2;
    double m_f0,m_f1,m_f2;
    const defaulttype::Vector3 & m_normal;

};

}

}
