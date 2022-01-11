#pragma once

#include <sofa/collisionAlgorithm/BaseProximity.h>

namespace sofa::collisionAlgorithm {

template<class DataTypes>
class TriangleProximity : public BaseProximity {
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

    TriangleProximity(State * state, unsigned p0,unsigned p1, unsigned p2, double f0,double f1,double f2)
    : m_state(state), m_p0(p0), m_p1(p1), m_p2(p2), m_f0(f0), m_f1(f1), m_f2(f2){}

    void buildJacobianConstraint(core::MultiMatrixDerivId cId, const sofa::type::vector<sofa::type::Vector3> & dir, double fact, Index constraintId) const override {
        DataMatrixDeriv & c1_d = *cId[m_state].write();
        MatrixDeriv & c1 = *c1_d.beginEdit();

        for (Index j=0;j<dir.size();j++) {
            MatrixDerivRowIterator c_it = c1.writeLine(constraintId+j);
            c_it.addCol(m_p0, dir[j] * m_f0 * fact);
            c_it.addCol(m_p1, dir[j] * m_f1 * fact);
            c_it.addCol(m_p2, dir[j] * m_f2 * fact);
        }

        c1_d.endEdit();
    }

    void storeLambda(const core::ConstraintParams* cParams, core::MultiVecDerivId resId, Index cid_global, Index cid_local, const sofa::linearalgebra::BaseVector* lambda) const override {
        auto res = sofa::helper::getWriteAccessor(*resId[m_state].write());
        const typename DataTypes::MatrixDeriv& j = cParams->readJ(m_state)->getValue();
        auto rowIt = j.readLine(cid_global+cid_local);
        const double f = lambda->element(cid_global+cid_local);
        for (auto colIt = rowIt.begin(), colItEnd = rowIt.end(); colIt != colItEnd; ++colIt)
        {
            res[colIt.index()] += colIt.val() * f;
        }
    }

    /// return proximiy position in a vector3
    sofa::type::Vector3 getPosition(core::VecCoordId v = core::VecCoordId::position()) const {
        const helper::ReadAccessor<DataVecCoord> & pos = m_state->read(v);
        sofa::type::Vector3 G = pos[m_p0] * m_f0 + pos[m_p1] * m_f1 + pos[m_p2] * m_f2;
        return G * 1.0/3.0;
    }

protected:
    State * m_state;
    unsigned m_p0,m_p1,m_p2;
    double m_f0,m_f1,m_f2;
};



template<class DataTypes>
class GouraudTriangleProximity : public TriangleProximity<DataTypes> {
public:
    typedef sofa::core::behavior::MechanicalState<DataTypes> State;
    typedef std::function<type::Vector3()> GetNormalFunc;

    GouraudTriangleProximity(State * state, unsigned p0,unsigned p1, unsigned p2, double f0,double f1,double f2, GetNormalFunc f)
    : TriangleProximity<DataTypes>(state,p0,p1,p2,f0,f1,f2)
    , m_func(f) {}

    virtual sofa::type::Vector3 getNormal() const override { return m_func(); }

private:
    GetNormalFunc m_func;
};



template<class DataTypes>
class PhongTriangleProximity : public TriangleProximity<DataTypes> {
public:
    typedef sofa::core::behavior::MechanicalState<DataTypes> State;

    PhongTriangleProximity(State * state, unsigned p0,unsigned p1, unsigned p2, double f0,double f1,double f2, type::vector<type::Vector3> & pn)
    : TriangleProximity<DataTypes>(state,p0,p1,p2,f0,f1,f2)
    , m_pointNormals(pn) {}

    virtual sofa::type::Vector3 getNormal() const override {
        return m_pointNormals[this->m_p0] * this->m_f0 +
               m_pointNormals[this->m_p1] * this->m_f1 +
               m_pointNormals[this->m_p2] * this->m_f2;
    }

private:
    const type::vector<type::Vector3> & m_pointNormals;
};


}
