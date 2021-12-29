#pragma once

#include <sofa/collisionAlgorithm/BaseProximity.h>

namespace sofa::collisionAlgorithm {

template<class DataTypes>
class PointProximity : public BaseProximity {
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

    void buildJacobianConstraint(core::MultiMatrixDerivId cId, const sofa::type::vector<sofa::type::Vector3> & dir, double fact, Index constraintId) const override {
        DataMatrixDeriv & c1_d = *cId[this->getState()].write();
        MatrixDeriv & c1 = *c1_d.beginEdit();

        for (Index j=0;j<dir.size();j++) {
            MatrixDerivRowIterator c_it = c1.writeLine(constraintId+j);
            c_it.addCol(m_pid, dir[j] * fact);
        }

        c1_d.endEdit();
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

