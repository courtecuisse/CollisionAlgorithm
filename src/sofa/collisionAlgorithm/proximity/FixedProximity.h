#pragma once

#include <sofa/collisionAlgorithm/BaseProximity.h>


namespace sofa
{

namespace collisionAlgorithm
{

//Default Proximity for fixed position
class FixedProximity : public BaseProximity {
public:

    FixedProximity(const defaulttype::Vector3 & p,const defaulttype::Vector3 & n = defaulttype::Vector3())
    : m_position(p), m_normal(n) {}

    static inline BaseProximity::SPtr create(const defaulttype::Vector3 & p,const defaulttype::Vector3 & n = defaulttype::Vector3()) {
        return BaseProximity::SPtr(new FixedProximity(p,n));
    }

    constexpr static CONTROL_POINT nbControlPoints() {
        return CONTROL_1;
    }

    defaulttype::Vector3 getPosition(core::VecCoordId ) const { return m_position; }

    virtual unsigned getElementId() const { return 0; }

    virtual defaulttype::Vector3 getNormal() const { return m_normal; }

    void buildJacobianConstraint(core::MultiMatrixDerivId /*cId*/, const helper::vector<defaulttype::Vector3> & /*m_normals*/, double /*fact*/, unsigned /*constraintId*/) const {}
    void buildConstraintMatrixJ0(int /*cId*/, sofa::defaulttype::BaseMatrix * /*J_from*/, double /*fact*/) const {}

    template<class MatrixDerivRowIterator>
    inline void addContributions(MatrixDerivRowIterator & it, const defaulttype::Vector3) const {}
    inline void addColInJ0(int /*cId*/, sofa::defaulttype::BaseMatrix * /*J0*/, double /*fact*/) const{}

    void storeLambda(const core::ConstraintParams* /*cParams*/, core::MultiVecDerivId /*res*/, unsigned /*cid_global*/, unsigned /*cid_local*/,const sofa::defaulttype::BaseVector* /*lambda*/) const {}

    defaulttype::Vector3 m_position;
    defaulttype::Vector3 m_normal;
};

}

}
