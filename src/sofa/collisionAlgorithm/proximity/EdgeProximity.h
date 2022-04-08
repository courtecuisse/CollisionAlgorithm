#pragma once

#include <sofa/collisionAlgorithm/BaseProximity.h>

namespace sofa::collisionAlgorithm {

class EdgeProximity : public BaseProximity {
public:

    EdgeProximity(BaseProximity::SPtr p0, BaseProximity::SPtr p1,double f0,double f1)
    : m_p0(p0), m_p1(p1), m_f0(f0), m_f1(f1) {}

    void buildJacobianConstraint(core::MultiMatrixDerivId cId, const sofa::type::vector<sofa::type::Vector3> & dir, double fact, Index constraintId) const override {
        sofa::type::vector<sofa::type::Vector3> N1;
        for (unsigned i=0;i<dir.size();i++) N1.push_back(dir[i]*m_f0);
        m_p0->buildJacobianConstraint(cId,N1,fact,constraintId);

        sofa::type::vector<sofa::type::Vector3> N2;
        for (unsigned i=0;i<dir.size();i++) N2.push_back(dir[i]*m_f1);
        m_p1->buildJacobianConstraint(cId,N2,fact,constraintId);
    }


    /// return proximiy position in a vector3
    sofa::type::Vector3 getPosition(core::VecCoordId v = core::VecCoordId::position()) const override {
        return m_p0->getPosition() * m_f0 +
               m_p1->getPosition() * m_f1;
    }

    sofa::type::Vector3 getNormal() const override {
        return m_p0->getNormal() * m_f0 +
               m_p1->getNormal() * m_f1;
    }

    void storeLambda(const core::ConstraintParams* cParams, core::MultiVecDerivId res, Index cid_global, Index cid_local, const sofa::defaulttype::BaseVector* lambda) const override {
        m_p0->storeLambda(cParams,res,cid_global,cid_local,lambda);
        m_p1->storeLambda(cParams,res,cid_global,cid_local,lambda);
    }

protected:
    BaseProximity::SPtr m_p0, m_p1;
    double m_f0,m_f1;

};

}
