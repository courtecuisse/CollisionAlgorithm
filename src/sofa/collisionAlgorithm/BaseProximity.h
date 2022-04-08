#pragma once

#include <sofa/core/VecId.h>
#include <sofa/core/MultiVecId.h>
#include <sofa/defaulttype/BaseVector.h>
#include <sofa/defaulttype/BaseMatrix.h>
#include <sofa/core/ConstraintParams.h>
#include <sofa/core/behavior/MechanicalState.h>
#include <sofa/core/topology/Topology.h>
#include <sofa/type/Vec.h>

namespace sofa::collisionAlgorithm {

/*!
 * \brief The BaseProximity class is the basic abstract proximity class
 */
class BaseProximity {
public:
    typedef std::shared_ptr<BaseProximity> SPtr;

    virtual ~BaseProximity() = default;

    /// return proximiy position in a vector3
    virtual sofa::type::Vector3 getPosition(core::VecCoordId v = core::VecCoordId::position()) const = 0;

    /// return normal in a vector3
    virtual sofa::type::Vector3 getNormal() const = 0;

    virtual void buildJacobianConstraint(core::MultiMatrixDerivId cId, const sofa::type::vector<sofa::type::Vector3> & dir, double fact, Index constraintId) const = 0;

    virtual void storeLambda(const core::ConstraintParams* cParams, core::MultiVecDerivId res, Index cid_global, Index cid_local, const sofa::defaulttype::BaseVector* lambda) const = 0;

    template<class PROXIMITY,class... ARGS>
    static inline typename PROXIMITY::SPtr create(ARGS... args) {
        return typename PROXIMITY::SPtr(new PROXIMITY(args...));
    }

};

/*!
 * Template implementation of BaseProximity
 */
template<class DataTypes>
class TBaseProximity : public BaseProximity {
public:
    typedef typename DataTypes::VecCoord VecCoord;
    typedef typename DataTypes::Coord Coord;
    typedef typename DataTypes::Real Real;
    typedef typename DataTypes::VecDeriv VecDeriv;
    typedef typename DataTypes::MatrixDeriv MatrixDeriv;
    typedef typename DataTypes::Deriv Deriv1;
    typedef typename MatrixDeriv::RowIterator MatrixDerivRowIterator;
    typedef core::objectmodel::Data< VecCoord >        DataVecCoord;
    typedef core::objectmodel::Data< VecDeriv >        DataVecDeriv;
    typedef core::objectmodel::Data< MatrixDeriv >     DataMatrixDeriv;
    typedef sofa::core::behavior::MechanicalState<DataTypes> State;

    virtual State * getState() const = 0;

    virtual void addContributions(MatrixDerivRowIterator & it, const sofa::type::Vector3 & N, double fact) const = 0;

    void buildJacobianConstraint(core::MultiMatrixDerivId cId, const sofa::type::vector<sofa::type::Vector3> & dir, double fact, Index constraintId) const override {
        DataMatrixDeriv & c1_d = *cId[getState()].write();
        MatrixDeriv & c1 = *c1_d.beginEdit();

        for (Index j=0;j<dir.size();j++) {
            MatrixDerivRowIterator c_it = c1.writeLine(constraintId+j);
            addContributions(c_it,dir[j],fact);
        }

        c1_d.endEdit();
    }

    void storeLambda(const core::ConstraintParams* cParams, core::MultiVecDerivId resId, Index cid_global, Index cid_local, const sofa::linearalgebra::BaseVector* lambda) const override {
        auto res = sofa::helper::getWriteAccessor(*resId[getState()].write());
        const typename DataTypes::MatrixDeriv& j = cParams->readJ(getState())->getValue();
        auto rowIt = j.readLine(cid_global+cid_local);
        const double f = lambda->element(cid_global+cid_local);
        for (auto colIt = rowIt.begin(), colItEnd = rowIt.end(); colIt != colItEnd; ++colIt)
        {
            res[colIt.index()] += colIt.val() * f;
        }
    }
};


}
