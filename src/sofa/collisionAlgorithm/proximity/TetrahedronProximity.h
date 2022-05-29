#pragma once

#include <sofa/collisionAlgorithm/elements/TetrahedronElement.h>

namespace sofa::collisionAlgorithm {

class TetrahedronProximity : public BaseProximity {
public:

    typedef std::shared_ptr<TetrahedronProximity> SPtr;

    TetrahedronProximity(TetrahedronElement::SPtr elmt, double f0,double f1,double f2,double f3)
    : m_elmt(elmt), m_f0(f0), m_f1(f1), m_f2(f2), m_f3(f3){}


    void buildJacobianConstraint(core::MultiMatrixDerivId cId, const sofa::type::vector<sofa::type::Vector3> & dir, double fact, Index constraintId) const override {
        sofa::type::vector<sofa::type::Vector3> N0;
        sofa::type::vector<sofa::type::Vector3> N1;
        sofa::type::vector<sofa::type::Vector3> N2;
        sofa::type::vector<sofa::type::Vector3> N3;

        for (unsigned i=0;i<dir.size();i++) {
            N0.push_back(dir[i]*m_f0);
            N1.push_back(dir[i]*m_f1);
            N2.push_back(dir[i]*m_f2);
            N3.push_back(dir[i]*m_f3);
        }
        m_elmt->getP0()->buildJacobianConstraint(cId,N0,fact,constraintId);
        m_elmt->getP1()->buildJacobianConstraint(cId,N1,fact,constraintId);
        m_elmt->getP2()->buildJacobianConstraint(cId,N2,fact,constraintId);
        m_elmt->getP3()->buildJacobianConstraint(cId,N3,fact,constraintId);

    }

//    std::vector<BaseProximity::SPtr> getProx() const {
//      return {m_p0,m_p1,m_p2,m_p3};
//    }

//	std::vector<double> getBaryCoord() const {
//		return {m_f0,m_f1,m_f2,m_f3};
//	}

    /// return proximiy position in a vector3
    sofa::type::Vector3 getPosition(core::VecCoordId v = core::VecCoordId::position()) const override {
        return m_elmt->getP0()->getPosition(v) * m_f0 +
               m_elmt->getP1()->getPosition(v) * m_f1 +
               m_elmt->getP2()->getPosition(v) * m_f2 +
               m_elmt->getP3()->getPosition(v) * m_f3;
    }

    void storeLambda(const core::ConstraintParams* cParams, core::MultiVecDerivId res, Index cid_global, Index cid_local, const sofa::linearalgebra::BaseVector* lambda) const override {
        m_elmt->getP0()->storeLambda(cParams,res,cid_global,cid_local,lambda);
        m_elmt->getP1()->storeLambda(cParams,res,cid_global,cid_local,lambda);
        m_elmt->getP2()->storeLambda(cParams,res,cid_global,cid_local,lambda);
        m_elmt->getP3()->storeLambda(cParams,res,cid_global,cid_local,lambda);
    }

    TetrahedronElement::SPtr element() { return m_elmt; }

protected:
    TetrahedronElement::SPtr m_elmt;
    double m_f0,m_f1,m_f2,m_f3;
};

}
