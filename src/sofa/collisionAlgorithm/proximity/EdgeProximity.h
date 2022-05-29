#pragma once

#include <sofa/collisionAlgorithm/BaseProximity.h>
#include <sofa/collisionAlgorithm/elements/EdgeElement.h>

namespace sofa::collisionAlgorithm {

class EdgeProximity : public BaseProximity {
public:

    typedef std::shared_ptr<EdgeProximity> SPtr;

    EdgeProximity(EdgeElement::SPtr elmt,double f0,double f1)
    : m_elmt(elmt), m_f0(f0), m_f1(f1) {}

    void buildJacobianConstraint(core::MultiMatrixDerivId cId, const sofa::type::vector<sofa::type::Vector3> & dir, double fact, Index constraintId) const override {
        sofa::type::vector<sofa::type::Vector3> N1;
        for (unsigned i=0;i<dir.size();i++) N1.push_back(dir[i]*m_f0);
        m_elmt->getP0()->buildJacobianConstraint(cId,N1,fact,constraintId);

        sofa::type::vector<sofa::type::Vector3> N2;
        for (unsigned i=0;i<dir.size();i++) N2.push_back(dir[i]*m_f1);
        m_elmt->getP1()->buildJacobianConstraint(cId,N2,fact,constraintId);
    }


    /// return proximiy position in a vector3
    sofa::type::Vector3 getPosition(core::VecCoordId v = core::VecCoordId::position()) const override {
        return m_elmt->getP0()->getPosition(v) * m_f0 +
               m_elmt->getP1()->getPosition(v) * m_f1;
    }

    void storeLambda(const core::ConstraintParams* cParams, core::MultiVecDerivId res, Index cid_global, Index cid_local, const sofa::defaulttype::BaseVector* lambda) const override {
        m_elmt->getP0()->storeLambda(cParams,res,cid_global,cid_local,lambda);
        m_elmt->getP1()->storeLambda(cParams,res,cid_global,cid_local,lambda);
    }

    EdgeElement::SPtr element() { return m_elmt; }

protected:
    EdgeElement::SPtr m_elmt;
    double m_f0,m_f1;
};

}
