#pragma once

#include <sofa/collisionAlgorithm/BaseProximity.h>

namespace sofa::collisionAlgorithm {

template<class DataTypes>
class MechanicalProximity : public TBaseProximity<DataTypes> {
public:

    typedef std::shared_ptr<MechanicalProximity<DataTypes> > SPtr;

    typedef TBaseProximity<DataTypes> PROXIMITY;
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


    MechanicalProximity(State * state,  unsigned pid)
    : m_state(state)
    , m_pid(pid) {}


    State * getState() const {
        return m_state;
    }

    void addContributions(MatrixDerivRowIterator & c_it, const sofa::type::Vector3 & N,double fact) const override {
        c_it.addCol(m_pid, N * fact);
    }

    /// return proximiy position in a vector3
    sofa::type::Vector3 getPosition(core::VecCoordId v = core::VecCoordId::position()) const {
        const helper::ReadAccessor<DataVecCoord> & pos = m_state->read(v);
        return pos[m_pid];

    }

    unsigned getPId() const {
        return m_pid;
    }


//    void storeLambda(const core::ConstraintParams* cParams, core::MultiVecDerivId resId, Index cid_global, Index cid_local, const sofa::defaulttype::BaseVector* lambda) const override {
//        auto res = sofa::helper::getWriteAccessor(*resId[getState()].write());
//        const typename DataTypes::MatrixDeriv& j = cParams->readJ(getState())->getValue();
//        auto rowIt = j.readLine(cid_global+cid_local);
//        const double f = lambda->element(cid_global+cid_local);
//        for (auto colIt = rowIt.begin(), colItEnd = rowIt.end(); colIt != colItEnd; ++colIt)
//        {
//            res[colIt.index()] += colIt.val() * f;
//        }
//    }



protected:
    State * m_state;
    unsigned m_pid;
};

}

