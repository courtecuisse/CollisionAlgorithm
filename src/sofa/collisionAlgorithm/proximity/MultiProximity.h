#pragma once

#include <sofa/collisionAlgorithm/BaseProximity.h>

namespace sofa::collisionAlgorithm {

class MultiProximity : public BaseProximity {
public:

    typedef std::shared_ptr<MultiProximity> SPtr;

    MultiProximity(std::vector<BaseProximity::SPtr> & prox)
    : m_proximities(prox){}

    /// return proximiy position in a vector3
    sofa::type::Vector3 getPosition(core::VecCoordId v) const {
        sofa::type::Vector3 P(0,0,0);
        for (unsigned i=0;i<m_proximities.size();i++) P+=m_proximities[i]->getPosition(v);
        return P*1.0/m_proximities.size();
    }

    void buildJacobianConstraint(core::MultiMatrixDerivId cId, const sofa::type::vector<sofa::type::Vector3> & dir, double fact, Index constraintId) const override {
        fact *= 1.0/m_proximities.size();
        for (unsigned i=0;i<m_proximities.size();i++) {
            m_proximities[i]->buildJacobianConstraint(cId,dir,fact,constraintId);
        }
    }

    void storeLambda(const core::ConstraintParams* cParams, core::MultiVecDerivId resId, Index cid_global, Index cid_local, const sofa::linearalgebra::BaseVector* lambda) const override {
        for (unsigned i=0;i<m_proximities.size();i++) {
            m_proximities[i]->storeLambda(cParams,resId,cid_global,cid_local,lambda);
        }
    }

    const std::type_info& getTypeInfo() const override { return typeid(MultiProximity); }

	virtual BaseProximity::SPtr copy() override
	{
		return SPtr(new MultiProximity(m_proximities));
	}

    bool isNormalized() const override { return true; }

    void normalize() override {}

protected:
    std::vector<BaseProximity::SPtr> m_proximities;
};

}
