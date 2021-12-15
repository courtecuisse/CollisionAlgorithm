#pragma once

#include <sofa/collisionAlgorithm/BaseProximity.h>


namespace sofa
{

namespace collisionAlgorithm
{

//Default Proximity for fixed position
class FixedProximity : public BaseProximity {
public:
    typedef BaseProximity::Index Index;

    FixedProximity(const sofa::type::Vector3 & p,const sofa::type::Vector3 & n = sofa::type::Vector3())
    : m_position(p), m_normal(n) {}

    static inline BaseProximity::SPtr create(const sofa::type::Vector3 & p,const sofa::type::Vector3 & n = sofa::type::Vector3()) {
        return BaseProximity::SPtr(new FixedProximity(p,n));
    }

    constexpr static CONTROL_POINT nbControlPoints() {
        return CONTROL_1;
    }

    sofa::type::Vector3 getPosition(core::VecCoordId ) const { return m_position; }

    virtual Index getElementId() const { return 0; }

    virtual sofa::type::Vector3 getNormal() const { return m_normal; }

    void buildJacobianConstraint(core::MultiMatrixDerivId /*cId*/, const sofa::type::vector<sofa::type::Vector3> & /*m_normals*/, double /*fact*/, Index /*constraintId*/) const {}

    template<class MatrixDerivRowIterator>
    inline void addContributions(MatrixDerivRowIterator & it, const sofa::type::Vector3) const {}

    void storeLambda(const core::ConstraintParams* /*cParams*/, core::MultiVecDerivId /*res*/, Index /*cid_global*/, Index /*cid_local*/,const sofa::linearalgebra::BaseVector* /*lambda*/) const {}

    sofa::type::Vector3 m_position;
    sofa::type::Vector3 m_normal;
};

}

}
