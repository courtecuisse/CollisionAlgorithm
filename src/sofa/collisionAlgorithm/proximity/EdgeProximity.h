#pragma once

#include <sofa/collisionAlgorithm/geometry/EdgeGeometry.h>

namespace sofa
{

namespace collisionAlgorithm
{

template<class DataTypes>
class EdgeProximity : public BaseProximity
{
public :
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
    typedef sofa::core::behavior::MechanicalState<DataTypes> State;

    EdgeProximity(unsigned p1,unsigned p2,double f1,double f2, State * state)
    : m_state(state) {
        m_pid[0] = p1;
        m_pid[1] = p2;
        m_fact[0] = f1;
        m_fact[1] = f2;
    }

    defaulttype::Vector3 getPosition(core::VecCoordId v) const
    {
        const helper::ReadAccessor<DataVecCoord> pos = m_state->read(v);
        return pos[m_pid[0]] * m_fact[0] + pos[m_pid[1]] * m_fact[1];
    }

    defaulttype::Vector3 getNormal() const
    {
        return defaulttype::Vector3(1,0,0);
    }

    void buildJacobianConstraint(core::MultiMatrixDerivId cId, const helper::vector<defaulttype::Vector3> & normals, double fact, unsigned constraintId) const {
        DataMatrixDeriv & c1_d = *cId[m_state].write();
        MatrixDeriv & c1 = *c1_d.beginEdit();

        for (unsigned j=0;j<normals.size();j++) {
            MatrixDerivRowIterator c_it = c1.writeLine(constraintId+j);

            c_it.addCol(m_pid[0], normals[j] * m_fact[0] * fact);
            c_it.addCol(m_pid[1], normals[j] * m_fact[1] * fact);
        }

        c1_d.endEdit();
    }

    sofa::core::behavior::BaseMechanicalState * getState() const { return m_state; }

    void storeLambda(const core::ConstraintParams* cParams, core::MultiVecDerivId res, const sofa::defaulttype::BaseVector* lambda) const {
        BaseProximity::TstoreLambda<DataTypes>(cParams, *res[m_state].write(), *cParams->readJ(m_state), lambda);
    }

protected:
    double m_pid[2];
    double m_fact[2];
    State * m_state;
};

}

}
