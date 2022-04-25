#pragma once

#include <sofa/collisionAlgorithm/BaseProximity.h>

namespace sofa::collisionAlgorithm {

class TriangleProximity : public BaseProximity{
public:

    TriangleProximity(BaseProximity::SPtr p0,BaseProximity::SPtr p1, BaseProximity::SPtr p2, double f0,double f1,double f2)
    : m_p0(p0), m_p1(p1), m_p2(p2), m_f0(f0), m_f1(f1), m_f2(f2){}


    void buildJacobianConstraint(core::MultiMatrixDerivId cId, const sofa::type::vector<sofa::type::Vector3> & dir, double fact, Index constraintId) const override {
        sofa::type::vector<sofa::type::Vector3> N0;
        sofa::type::vector<sofa::type::Vector3> N1;
        sofa::type::vector<sofa::type::Vector3> N2;

        for (unsigned i=0;i<dir.size();i++) {
            N0.push_back(dir[i]*m_f0);
            N1.push_back(dir[i]*m_f1);
            N2.push_back(dir[i]*m_f2);
        }

        m_p0->buildJacobianConstraint(cId,N0,fact,constraintId);
        m_p1->buildJacobianConstraint(cId,N1,fact,constraintId);
        m_p2->buildJacobianConstraint(cId,N2,fact,constraintId);

    }

//    void addContributions(MatrixDerivRowIterator & c_it, const sofa::type::Vector3 & N,double fact) const override {
//        c_it.addCol(m_p0, N * m_f0 * fact);
//        c_it.addCol(m_p1, N * m_f1 * fact);
//        c_it.addCol(m_p2, N * m_f2 * fact);
//    }

    /// return proximiy position in a vector3
    sofa::type::Vector3 getPosition(core::VecCoordId v = core::VecCoordId::position()) const {
        return m_p0->getPosition(v) * m_f0 +
               m_p1->getPosition(v) * m_f1 +
               m_p2->getPosition(v) * m_f2;
    }

    sofa::type::Vector3 getNormal() const override {
        return m_p0->getNormal() * m_f0 +
               m_p1->getNormal() * m_f1 +
               m_p2->getNormal() * m_f2;
    }

    void storeLambda(const core::ConstraintParams* cParams, core::MultiVecDerivId res, Index cid_global, Index cid_local, const sofa::defaulttype::BaseVector* lambda) const override {
        m_p0->storeLambda(cParams,res,cid_global,cid_local,lambda);
        m_p1->storeLambda(cParams,res,cid_global,cid_local,lambda);
        m_p2->storeLambda(cParams,res,cid_global,cid_local,lambda);
    }

protected:
    BaseProximity::SPtr m_p0,m_p1,m_p2;
    double m_f0,m_f1,m_f2;
};

}
