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

class MBaseIterator {
public:

    MBaseIterator(sofa::defaulttype::BaseMatrix * M, unsigned int row) : m_J(M), m_row(row) {}

    void addCol(unsigned c, const defaulttype::Vector3 & N) {
        if(N[1] == 0.0){
            double v = N[0];
            m_J->add(m_row, c, v);
        }
        else{
            double v = N[0];
            m_J->add(m_row*3, c*3, v);
            m_J->add(m_row*3+1, c*3+1, v);
            m_J->add(m_row*3+2, c*3+2, v);
        }

    }

private:
    sofa::defaulttype::BaseMatrix * m_J;
    unsigned int m_row;

};

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

    virtual void storeLambda(const core::ConstraintParams* cParams, core::MultiVecDerivId res, unsigned cid_global, unsigned cid_local, const sofa::defaulttype::BaseVector* lambda) const = 0;

    virtual unsigned getElementId() const = 0;

    virtual void buildConstraintProximityMatrix(int cId, sofa::defaulttype::BaseMatrix * J_prox, double fact, const bool expand)const {
        // temporary : leave this empty please
    }

    virtual sofa::core::behavior::MechanicalState<defaulttype::Vec3Types> * getState() const{
        // temporary : leave this empty please
    }

};

/*!
 * Template implementation of BaseProximity
 */
template<class GEOMETRY, class PROXIMITYDATA>
class TBaseProximity : public BaseProximity {
public:
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

    defaulttype::Vector3 getPosition(core::VecCoordId v = core::VecCoordId::position()) const override {
        return m_geometry->getPosition(m_data,v);
    }

    defaulttype::Vector3 getNormal() const override {
        return m_geometry->getNormal(m_data);
    }

    void buildJacobianConstraint(core::MultiMatrixDerivId cId, const helper::vector<defaulttype::Vector3> & normals, double fact, unsigned constraintId) const {
        m_geometry->buildJacobianConstraint(m_data,cId,normals,fact,constraintId);
    }

    void storeLambda(const core::ConstraintParams* cParams, core::MultiVecDerivId resId, unsigned cid_global, unsigned cid_local, const sofa::defaulttype::BaseVector* lambda) const {
        m_geometry->storeLambda(cParams,resId,cid_global,cid_local, lambda);
    }

    inline unsigned getElementId() const override {
        return m_data.getElementId();
    }

    const PROXIMITYDATA& getProximityData() const {
        return m_data;
    }

    void buildConstraintProximityMatrix(int cId, sofa::defaulttype::BaseMatrix * J_prox, double fact, const bool expand) const{
        MBaseIterator J_iterator(J_prox, cId);
        defaulttype::Vector3 N(1,0,0);
        if(expand) N = defaulttype::Vector3(1,1,1);
        m_data.addContributions(J_iterator, N*fact);
    }

    sofa::core::behavior::MechanicalState<defaulttype::Vec3Types> * getState() const{
        sofa::core::behavior::MechanicalState<defaulttype::Vec3Types> *ms  = dynamic_cast<sofa::core::behavior::MechanicalState<defaulttype::Vec3Types> *> (m_geometry->getState());
        return ms;
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
