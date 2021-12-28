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
    : m_state(state), m_p0(p0), m_p1(p1), m_p2(p2), m_f0(f0), m_f1(f1), m_f2(f2) {}

    void buildJacobianConstraint(core::MultiMatrixDerivId cId, const sofa::type::vector<sofa::type::Vector3> & dir, double fact, Index constraintId) const override {
        for (Index j=0;j<dir.size();j++) {
            std::vector<sofa::type::Vector3> N0,N1,N2;

            N0.push_back(dir[j] * m_f0);
            N1.push_back(dir[j] * m_f1);
            N2.push_back(dir[j] * m_f2);

            m_p0->buildJacobianConstraint(cId,N0,fact,constraintId + j);
            m_p1->buildJacobianConstraint(cId,N1,fact,constraintId + j);
            m_p2->buildJacobianConstraint(cId,N2,fact,constraintId + j);
        }
    }

    virtual sofa::type::Vector3 getNormal() const {
        sofa::type::Vector3 G = m_p0->getNormal() * m_f0 +
                                m_p1->getNormal() * m_f1 +
                                m_p2->getNormal() * m_f2;
        return G * 1.0/3.0;
    }

    /// return proximiy position in a vector3
    sofa::type::Vector3 getPosition(core::VecCoordId v = core::VecCoordId::position()) const {
        sofa::type::Vector3 G = m_p0->getPosition(v) * m_f0 +
                                m_p1->getPosition(v) * m_f1 +
                                m_p2->getPosition(v) * m_f2;
        return G * 1.0/3.0;
    }

private:
    State * m_state;
    unsigned m_p0,m_p1,m_p2;
    double m_f0,m_f1,m_f2;
};

}
