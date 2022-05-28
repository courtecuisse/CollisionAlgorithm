#pragma once

#include <sofa/collisionAlgorithm/elements/PointElement.h>

namespace sofa::collisionAlgorithm {

class PointProximity : public BaseProximity {
public:

    PointProximity(PointElement::SPtr elmt)
    : m_elmt(elmt) {}

    void buildJacobianConstraint(core::MultiMatrixDerivId cId, const sofa::type::vector<sofa::type::Vector3> & dir, double fact, Index constraintId) const override {
        m_elmt->getP0()->buildJacobianConstraint(cId,dir,fact,constraintId);
    }

    /// return proximiy position in a vector3
    sofa::type::Vector3 getPosition(core::VecCoordId v = core::VecCoordId::position()) const {
        return m_elmt->getP0()->getPosition(v);
    }

    void storeLambda(const core::ConstraintParams* cParams, core::MultiVecDerivId res, Index cid_global, Index cid_local, const sofa::defaulttype::BaseVector* lambda) const override {
        m_elmt->getP0()->storeLambda(cParams,res,cid_global,cid_local,lambda);
    }

protected:
    PointElement::SPtr m_elmt;
};

}

