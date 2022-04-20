#pragma once

#include <sofa/collisionAlgorithm/BaseProximity.h>

namespace sofa::collisionAlgorithm {

template<class DataTypes>
class TopologyProximity : public TBaseProximity<DataTypes> {
public:

    typedef std::shared_ptr<TopologyProximity<DataTypes> > SPtr;

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


    TopologyProximity(State * state, unsigned pid)
    : m_state(state)
    , m_pid(pid)
    {
        m_normal = type::Vector3();
    }


    State * getState() const {
           return m_state;
       }

//    void buildJacobianConstraint(core::MultiMatrixDerivId cId, const sofa::type::vector<sofa::type::Vector3> & dir, double fact, Index constraintId) const override {
//        DataMatrixDeriv & c1_d = *cId[getState()].write();
//        MatrixDeriv & c1 = *c1_d.beginEdit();

//        for (Index j=0;j<dir.size();j++) {
//            MatrixDerivRowIterator c_it = c1.writeLine(constraintId+j);
//            addContributions(c_it,dir[j],fact);
//        }

//        c1_d.endEdit();



//    }

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

    sofa::type::Vector3 getNormal() const override {
        return m_normal;
    }

    void setNormal(sofa::type::Vector3 normal) {
        m_normal = normal;
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
    sofa::type::Vector3 m_normal;
};

}

