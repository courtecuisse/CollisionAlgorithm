#pragma once

#include <sofa/collisionAlgorithm/BaseProximity.h>

namespace sofa
{

namespace collisionAlgorithm
{

//Default Proximity for fixed position
class FixedProximity : public BaseProximity {
public:

    FixedProximity(const defaulttype::Vector3 & p)
    : BaseProximity(0), m_position(p) {}

    defaulttype::Vector3 getPosition(core::VecCoordId ) const {
        return m_position;
    }

    virtual defaulttype::Vector3 getNormal() const {
        return defaulttype::Vector3();
    }

    void buildJacobianConstraint(core::MultiMatrixDerivId /*cId*/, const helper::vector<defaulttype::Vector3> & /*m_normals*/, double /*fact*/, unsigned /*constraintId*/) const {}

    void storeLambda(const core::ConstraintParams* /*cParams*/, core::MultiVecDerivId /*res*/, unsigned /*cid*/, const sofa::defaulttype::BaseVector* /*lambda*/) const {}

    defaulttype::Vector3 m_position;
};

}

}
