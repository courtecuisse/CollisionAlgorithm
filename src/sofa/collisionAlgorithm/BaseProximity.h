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

class BaseGeometry;

class BaseProximity {
public :
    typedef std::shared_ptr<BaseProximity> SPtr;
    typedef Data<helper::vector<defaulttype::Vector3> > DataVecCoord;

    virtual defaulttype::Vector3 getPosition(core::VecCoordId v = core::VecCoordId::position()) const = 0;

    virtual defaulttype::Vector3 getNormal() const = 0;

    virtual void buildJacobianConstraint(core::MultiMatrixDerivId cId, const helper::vector<defaulttype::Vector3> & dir, double fact, unsigned constraintId) const = 0;

    virtual void storeLambda(const core::ConstraintParams* cParams, core::MultiVecDerivId res, unsigned cid, const sofa::defaulttype::BaseVector* lambda) const = 0;

    template<class PROXIMITY, class ... Args>
    static BaseProximity::SPtr create(Args&& ... args) {
        return SPtr(new PROXIMITY(std::forward<Args>(args)...));
    }
};

template<class GEOMETRY>
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

    TBaseProximity(const GEOMETRY * geo)
    : m_geometry(geo) {}

    virtual void addContributions(MatrixDerivRowIterator & it, const defaulttype::Vector3 & N) const = 0;

    void buildJacobianConstraint(core::MultiMatrixDerivId cId, const helper::vector<defaulttype::Vector3> & normals, double fact, unsigned constraintId) const {
        DataMatrixDeriv & c1_d = *cId[m_geometry->getState()].write();
        MatrixDeriv & c1 = *c1_d.beginEdit();

        for (unsigned j=0;j<normals.size();j++) {
            MatrixDerivRowIterator c_it = c1.writeLine(constraintId+j);
            addContributions(c_it, normals[j] * fact);
        }

        c1_d.endEdit();
    }

    virtual void storeLambda(const core::ConstraintParams* cParams, core::MultiVecDerivId resId, unsigned cid, const sofa::defaulttype::BaseVector* lambda) const {
        auto res = sofa::helper::write(*resId[m_geometry->getState()].write(), cParams);
        const typename DataTypes::MatrixDeriv& j = cParams->readJ(m_geometry->getState())->getValue();
        auto rowIt = j.readLine(cid);
        const SReal f = lambda->element(cid);
        for (auto colIt = rowIt.begin(), colItEnd = rowIt.end(); colIt != colItEnd; ++colIt)
        {
            res[colIt.index()] += colIt.val() * f;
        }
    }


protected:
    const GEOMETRY * m_geometry;

};

}

}
