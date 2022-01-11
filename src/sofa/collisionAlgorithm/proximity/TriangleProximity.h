#pragma once

#include <sofa/collisionAlgorithm/BaseProximity.h>

namespace sofa::collisionAlgorithm {

template<class DataTypes>
class BaseTriangleProximity : public TBaseProximity<DataTypes> {
public:
    typedef sofa::core::behavior::MechanicalState<DataTypes> State;
    typedef typename DataTypes::VecCoord VecCoord;
    typedef typename DataTypes::Coord Coord;
    typedef typename DataTypes::Real Real;
    typedef typename DataTypes::VecDeriv VecDeriv;
    typedef typename DataTypes::MatrixDeriv MatrixDeriv;
    typedef typename DataTypes::Deriv Deriv1;
    typedef typename MatrixDeriv::RowIterator MatrixDerivRowIterator;
    typedef core::objectmodel::Data< VecCoord >        DataVecCoord;
    typedef core::objectmodel::Data< VecDeriv >        DataVecDeriv;
    typedef core::objectmodel::Data< MatrixDeriv >     DataMatrixDeriv;

    BaseTriangleProximity(State * state, unsigned p0,unsigned p1, unsigned p2, double f0,double f1,double f2)
    : m_state(state), m_p0(p0), m_p1(p1), m_p2(p2), m_f0(f0), m_f1(f1), m_f2(f2){}

    State * getState() const {
        return m_state;
    }

    void addContributions(MatrixDerivRowIterator & c_it, const sofa::type::Vector3 & N,double fact) const override {
        c_it.addCol(m_p0, N * m_f0 * fact);
        c_it.addCol(m_p1, N * m_f1 * fact);
        c_it.addCol(m_p2, N * m_f2 * fact);
    }

    /// return proximiy position in a vector3
    sofa::type::Vector3 getPosition(core::VecCoordId v = core::VecCoordId::position()) const {
        const helper::ReadAccessor<DataVecCoord> & pos = m_state->read(v);
        return (pos[m_p0] * m_f0 + pos[m_p1] * m_f1 + pos[m_p2] * m_f2) * 1.0/3.0;
    }

protected:
    State * m_state;
    unsigned m_p0,m_p1,m_p2;
    double m_f0,m_f1,m_f2;
};

}
