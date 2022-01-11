#pragma once

#include <sofa/collisionAlgorithm/BaseProximity.h>

namespace sofa {

namespace collisionAlgorithm {

template<class DataTypes>
class EdgeProximity {
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

    EdgeProximity(State * s, unsigned p0,unsigned p1,double f0,double f1,std::function<type::Vector3()> f)
    : m_state(s), m_p0(p0), m_p1(p1), m_f0(f0), m_f1(f1) {}

    State * getState() const {
        return m_state;
    }

    void addContributions(MatrixDerivRowIterator & c_it, const sofa::type::Vector3 & N,double fact) const override {
        c_it.addCol(m_p0, N * m_f0 * fact);
        c_it.addCol(m_p1, N * m_f1 * fact);
    }

    sofa::type::Vector3 getNormal() const override { return getNormalFunc(); }

    /// return proximiy position in a vector3
    sofa::type::Vector3 getPosition(core::VecCoordId v = core::VecCoordId::position()) const {
        const helper::ReadAccessor<DataVecCoord> & pos = m_state->read(v);

        sofa::type::Vector3 G = pos[m_p0] * m_f0 + pos[m_p1] * m_f1;
        return G * 1.0/3.0;
    }

private:
    State * m_state;
    unsigned m_p0, m_p1;
    double m_f0,m_f1;
    std::function<type::Vector3()> getNormalFunc;

};

}  // namespace collisionAlgorithm

}  // namespace sofa
