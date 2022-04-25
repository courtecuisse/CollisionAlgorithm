#pragma once

#include <sofa/collisionAlgorithm/BaseProximity.h>

namespace sofa::collisionAlgorithm {

class PointProximity : public BaseProximity {
public:

    PointProximity(BaseProximity::SPtr prox)
    : m_p0(prox) {}

    void buildJacobianConstraint(core::MultiMatrixDerivId cId, const sofa::type::vector<sofa::type::Vector3> & dir, double fact, Index constraintId) const override {
        m_p0->buildJacobianConstraint(cId,dir,fact,constraintId);
    }

//    void addContributions(MatrixDerivRowIterator & c_it, const sofa::type::Vector3 & N,double fact) const override {
//        c_it.addCol(m_p0, N * fact);
//    }

    /// return proximiy position in a vector3
    sofa::type::Vector3 getPosition(core::VecCoordId v = core::VecCoordId::position()) const {
        return m_p0->getPosition(v);
    }

    BaseProximity::SPtr getProx() const {
        return m_p0;
    }

    sofa::type::Vector3 getNormal() const override {
        return m_p0->getNormal();
    }

    void storeLambda(const core::ConstraintParams* cParams, core::MultiVecDerivId res, Index cid_global, Index cid_local, const sofa::defaulttype::BaseVector* lambda) const override {
        m_p0->storeLambda(cParams,res,cid_global,cid_local,lambda);
    }

protected:
    BaseProximity::SPtr m_p0;
};

}

