#pragma once

#include <sofa/collisionAlgorithm/BaseProximity.h>

namespace sofa::collisionAlgorithm {

template<class DataTypes>
class PointProximity : public TBaseProximity<DataTypes> {
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

    PointProximity(State * s, unsigned pid,std::function<type::Vector3()> f)
    : m_state(s), m_pid(pid), getNormalFunc(f) {}

    State * getState() const {
        return m_state;
    }

    void addContributions(MatrixDerivRowIterator & c_it, const sofa::type::Vector3 & N,double fact) const override {
        c_it.addCol(m_pid, N * fact);
    }

    sofa::type::Vector3 getNormal() const override { return getNormalFunc(); }

    /// return proximiy position in a vector3
    sofa::type::Vector3 getPosition(core::VecCoordId v = core::VecCoordId::position()) const {
        const helper::ReadAccessor<DataVecCoord> & pos = m_state->read(v);
        return pos[m_pid];
    }


private:
    State * m_state;
    unsigned m_pid;
    std::function<type::Vector3()> getNormalFunc;
};

}

