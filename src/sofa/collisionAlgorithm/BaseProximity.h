#pragma once

#include <memory>
#include <map>
#include <vector>
#include <sofa/core/VecId.h>
#include <sofa/core/MultiVecId.h>
#include <sofa/defaulttype/BaseVector.h>
#include <sofa/core/ConstraintParams.h>

namespace sofa
{

namespace collisionAlgorithm
{

class BaseElement;
class BaseGeometry;

class BaseProximity {
public :
    typedef std::shared_ptr<BaseProximity> SPtr;
    typedef Data<helper::vector<defaulttype::Vector3> > DataVecCoord;

    virtual defaulttype::Vector3 getPosition(core::VecCoordId v = core::VecCoordId::position()) const = 0;

    virtual defaulttype::Vector3 getNormal() const = 0;

    virtual void buildJacobianConstraint(core::MultiMatrixDerivId cId, const helper::vector<defaulttype::Vector3> & dir, double fact, unsigned constraintId) const = 0;

    virtual sofa::core::behavior::BaseMechanicalState * getState() const = 0;

    virtual void storeLambda(const core::ConstraintParams* cParams, core::MultiVecDerivId res, const sofa::defaulttype::BaseVector* lambda) const = 0;

protected:
    template<class DataTypes>
    static void TstoreLambda(const core::ConstraintParams* cParams, Data<typename DataTypes::VecDeriv>& result, const Data<typename DataTypes::MatrixDeriv>& jacobian, const sofa::defaulttype::BaseVector* lambda) {
        auto res = sofa::helper::write(result, cParams);
        const typename DataTypes::MatrixDeriv& j = jacobian.getValue(cParams);
        j.multTransposeBaseVector(res, lambda ); // lambda is a vector of scalar value so block size is one.
    }

};

//Default Proximity for fixed position
class FixedProximity : public BaseProximity {
public:

    FixedProximity(defaulttype::Vector3 & p) : m_position(p) {}

    defaulttype::Vector3 getPosition(core::VecCoordId ) const {
        return m_position;
    }

    virtual defaulttype::Vector3 getNormal() const {
        return defaulttype::Vector3();
    }

    void buildJacobianConstraint(core::MultiMatrixDerivId /*cId*/, const helper::vector<defaulttype::Vector3> & /*m_normals*/, double /*fact*/, unsigned /*constraintId*/) const {}

    void storeLambda(const core::ConstraintParams* /*cParams*/, core::MultiVecDerivId /*res*/, const sofa::defaulttype::BaseVector* /*lambda*/) const {}

    sofa::core::behavior::BaseMechanicalState * getState() const { return NULL; }

    defaulttype::Vector3 m_position;
};

}

}
