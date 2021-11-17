#pragma once

#include <sofa/core/VecId.h>
#include <sofa/core/MultiVecId.h>
#include <sofa/defaulttype/BaseVector.h>
#include <sofa/defaulttype/BaseMatrix.h>
#include <sofa/core/ConstraintParams.h>
#include <sofa/core/behavior/MechanicalState.h>
#include <sofa/core/topology/Topology.h>
#include <sofa/type/Vec.h>

namespace sofa
{

namespace collisionAlgorithm
{

enum CONTROL_POINT {
    CONTROL_DEFAULT = -1,
    CONTROL_0 = 0,
    CONTROL_1 = 1,
    CONTROL_2 = 2,
    CONTROL_3 = 3,
    CONTROL_4 = 4,
    CONTROL_5 = 5,
    CONTROL_6 = 6,
    CONTROL_7 = 7,
    CONTROL_8 = 8,
};

/*!
 * \brief The BaseProximity class is the basic abstract proximity class
 */
class BaseProximity {
public :
    typedef sofa::Index Index;

    typedef std::shared_ptr<BaseProximity> SPtr;

    /// return proximiy position in a vector3
    virtual sofa::type::Vector3 getPosition(core::VecCoordId v = core::VecCoordId::position()) const = 0;

    /// return normal in a vector3
    virtual sofa::type::Vector3 getNormal() const = 0;

    virtual void buildJacobianConstraint(core::MultiMatrixDerivId cId, const sofa::type::vector<sofa::type::Vector3> & dir, double fact, Index constraintId) const = 0;

    virtual void storeLambda(const core::ConstraintParams* cParams, core::MultiVecDerivId res, Index cid_global, Index cid_local, const sofa::defaulttype::BaseVector* lambda) const = 0;

    virtual Index getElementId() const = 0;

};

/*!
 * Template implementation of BaseProximity
 */
template<class GEOMETRY, class PROXIMITYDATA>
class TBaseProximity : public BaseProximity {
public:
    typedef BaseProximity::Index Index;
    typedef typename GEOMETRY::TDataTypes DataTypes;
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

    TBaseProximity(const GEOMETRY * container, const PROXIMITYDATA & data)
    : m_geometry(container)
    , m_data(data) {}

    sofa::type::Vector3 getPosition(core::VecCoordId v = core::VecCoordId::position()) const override {
        return m_geometry->getPosition(m_data,v);
    }

    sofa::type::Vector3 getNormal() const override {
        return m_geometry->getNormal(m_data);
    }

    void buildJacobianConstraint(core::MultiMatrixDerivId cId, const sofa::type::vector<sofa::type::Vector3> & normals, double fact, Index constraintId) const {
        m_geometry->buildJacobianConstraint(m_data,cId,normals,fact,constraintId);
    }

    void storeLambda(const core::ConstraintParams* cParams, core::MultiVecDerivId resId, Index cid_global, Index cid_local, const sofa::defaulttype::BaseVector* lambda) const {
        m_geometry->storeLambda(cParams,resId,cid_global,cid_local, lambda);
    }

    inline Index getElementId() const override {
        return m_data.getElementId();
    }

    const PROXIMITYDATA& getProximityData() const {
        return m_data;
    }

protected:
    const GEOMETRY * m_geometry;
    const PROXIMITYDATA m_data;
};

template<class GEOMETRY, class PROXIMITYDATA>
static BaseProximity::SPtr createProximity(const GEOMETRY * container, const PROXIMITYDATA & data) {
    return BaseProximity::SPtr(new TBaseProximity<GEOMETRY,PROXIMITYDATA>(container, data));
}

}

}
