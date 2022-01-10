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

//    virtual sofa::type::Vector3 getNormal() const override {
////        return getNormalFunc(m_p0,m_p1,m_p2,m_f0,m_f1,m_f2,);
//    }

    /// return proximiy position in a vector3
    sofa::type::Vector3 getPosition(core::VecCoordId v = core::VecCoordId::position()) const {
        const helper::ReadAccessor<DataVecCoord> & pos = m_state->read(v);
        sofa::type::Vector3 G = pos[m_p0] * m_f0 + pos[m_p1] * m_f1 + pos[m_p2] * m_f2;
        return G * 1.0/3.0;
    }

private:
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


}
