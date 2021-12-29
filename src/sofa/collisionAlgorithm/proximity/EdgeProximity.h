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

    void buildJacobianConstraint(core::MultiMatrixDerivId cId, const sofa::type::vector<sofa::type::Vector3> & dir, double fact, Index constraintId) const override {
        DataMatrixDeriv & c1_d = *cId[m_state].write();
        MatrixDeriv & c1 = *c1_d.beginEdit();

        for (Index j=0;j<dir.size();j++) {
            MatrixDerivRowIterator c_it = c1.writeLine(constraintId+j);
            c_it.addCol(m_p0, dir[j] * m_f0);
            c_it.addCol(m_p1, dir[j] * m_f1);
        }

        c1_d.endEdit();
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
