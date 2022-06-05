#pragma once

#include <sofa/core/VecId.h>
#include <sofa/core/MultiVecId.h>
#include <sofa/linearalgebra/BaseVector.h>
#include <sofa/core/ConstraintParams.h>
#include <sofa/core/behavior/MechanicalState.h>
#include <sofa/core/topology/Topology.h>
#include <sofa/type/Vec.h>

namespace sofa::collisionAlgorithm {

/*!
 * \brief The BaseProximity class is the basic abstract proximity class
 */
class BaseElement;

class BaseProximity {
public:
    typedef std::shared_ptr<BaseProximity> SPtr;

    virtual ~BaseProximity() = default;

    /// return proximiy position in a vector3
    virtual sofa::type::Vector3 getPosition(core::VecCoordId v = core::VecCoordId::position()) const = 0;

    virtual void buildJacobianConstraint(core::MultiMatrixDerivId cId, const sofa::type::vector<sofa::type::Vector3> & dir, double fact, Index constraintId) const = 0;

    virtual void storeLambda(const core::ConstraintParams* cParams, core::MultiVecDerivId res, Index cid_global, Index cid_local, const sofa::linearalgebra::BaseVector* lambda) const = 0;

    virtual const std::type_info& getTypeInfo() const = 0;

    template<class PROXIMITY,class... ARGS>
    static inline typename PROXIMITY::SPtr create(ARGS... args) {
        return typename PROXIMITY::SPtr(new PROXIMITY(args...));
    }
};

}
