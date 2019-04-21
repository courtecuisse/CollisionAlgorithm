#pragma once

#include <sofa/core/VecId.h>
#include <sofa/core/MultiVecId.h>
#include <sofa/defaulttype/BaseVector.h>
#include <sofa/core/ConstraintParams.h>
#include <sofa/core/behavior/MechanicalState.h>

namespace sofa
{

namespace collisionAlgorithm
{

/*!
 * \brief The BaseProximity class is the basic abstract proximity class
 */
class BaseProximity {
public :
    typedef std::shared_ptr<BaseProximity> SPtr;

    /// return proximiy position in a vector3
    virtual defaulttype::Vector3 getPosition(core::VecCoordId v = core::VecCoordId::position()) const = 0;

    /// return normal in a vector3
    virtual defaulttype::Vector3 getNormal() const = 0;

    virtual void buildJacobianConstraint(core::MultiMatrixDerivId cId, const helper::vector<defaulttype::Vector3> & dir, double fact, unsigned constraintId) const = 0;

    virtual void storeLambda(const core::ConstraintParams* cParams, core::MultiVecDerivId res, unsigned cid, const sofa::defaulttype::BaseVector* lambda) const = 0;
};

/*!
 * Template implementation of BaseProximity
 */
template<class DataTypes, class PROXIMITYDATA>
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
    typedef std::function<defaulttype::Vector3(const PROXIMITYDATA & ,core::VecCoordId )> PositionFunctor;
    typedef std::function<defaulttype::Vector3(const PROXIMITYDATA & )> NormalFunctor;

    TBaseProximity(State * state, const PROXIMITYDATA & data, PositionFunctor fp, NormalFunctor fn)
    : m_state(state)
    , m_data(data)
    , m_positionFunctor(fp)
    , m_normalFunctor(fn) {}

    defaulttype::Vector3 getPosition(core::VecCoordId v = core::VecCoordId::position()) const {
        return m_positionFunctor(m_data,v);
    }

    defaulttype::Vector3 getNormal() const {
        return m_normalFunctor(m_data);
    }

    virtual void buildJacobianConstraint(core::MultiMatrixDerivId cId, const helper::vector<defaulttype::Vector3> & normals, double fact, unsigned constraintId) const {
        DataMatrixDeriv & c1_d = *cId[m_state].write();
        MatrixDeriv & c1 = *c1_d.beginEdit();

        for (unsigned j=0;j<normals.size();j++) {
            MatrixDerivRowIterator c_it = c1.writeLine(constraintId+j);
            m_data.addContributions(c_it, normals[j] * fact);
        }

        c1_d.endEdit();
    }

    virtual void storeLambda(const core::ConstraintParams* cParams, core::MultiVecDerivId resId, unsigned cid, const sofa::defaulttype::BaseVector* lambda) const {
        auto res = sofa::helper::write(*resId[m_state].write(), cParams);
        const typename DataTypes::MatrixDeriv& j = cParams->readJ(m_state)->getValue();
        auto rowIt = j.readLine(cid);
        const double f = lambda->element(cid);
        for (auto colIt = rowIt.begin(), colItEnd = rowIt.end(); colIt != colItEnd; ++colIt)
        {
            res[colIt.index()] += colIt.val() * f;
        }
    }



protected:
    State * m_state;
    const PROXIMITYDATA m_data;
    const PositionFunctor m_positionFunctor;
    const NormalFunctor m_normalFunctor;
};

}

}
